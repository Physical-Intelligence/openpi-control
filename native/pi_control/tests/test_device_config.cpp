#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <string>

#include "pi_command_line_args.hpp"
#include "pi_device_config.hpp"

namespace {

class DeviceConfigTest : public ::testing::Test {
   protected:
    void SetUp() override {
        dir_ = std::filesystem::temp_directory_path() /
               ("pi_device_config_test_" +
                std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
                ::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::filesystem::create_directories(dir_);
    }

    void TearDown() override { std::filesystem::remove_all(dir_); }

    std::string write_config(const std::string& filename, const std::string& content) {
        const auto path = dir_ / filename;
        std::ofstream file(path);
        file << content;
        return path.string();
    }

    CommandLineArgs make_arm_cla(const std::string& model_config_path) {
        CommandLineArgs cla;
        cla.device_config_type = DeviceConfigType::ARM;
        cla.arm_model_config = model_config_path;
        return cla;
    }

    std::filesystem::path dir_;
};

}  // namespace

TEST_F(DeviceConfigTest, LoadsExplicitModelConfigPath) {
    const auto path = write_config("model.json", R"({
        "config_version": "1.1.1",
        "device_model": "Yam",
        "device_type": "arm"
    })");
    DeviceConfig config;
    ASSERT_EQ(config.init_config_model(make_arm_cla(path)), ReturnCode::SUCCESS);

    std::string model;
    EXPECT_EQ(config.get_field_value(config.values_, config.fn_device_model, model), ReturnCode::SUCCESS);
    EXPECT_EQ(model, "Yam");
}

TEST_F(DeviceConfigTest, MissingFileIsNotFound) {
    DeviceConfig config;
    EXPECT_EQ(config.init_config_model(make_arm_cla((dir_ / "does_not_exist.json").string())),
              ReturnCode::NOT_FOUND);
}

TEST_F(DeviceConfigTest, MalformedJsonIsRejectedNotThrown) {
    const auto path = write_config("model.json", "{ this is not json");
    DeviceConfig config;
    EXPECT_EQ(config.init_config_model(make_arm_cla(path)), ReturnCode::INVALID_PARAM);
}

TEST_F(DeviceConfigTest, UnsupportedConfigVersionIsRejected) {
    const auto path = write_config("model.json", R"({"config_version": "9.9.9"})");
    DeviceConfig config;
    EXPECT_EQ(config.init_config_model(make_arm_cla(path)), ReturnCode::NOT_SUPPORTED);
}

TEST_F(DeviceConfigTest, MissingConfigVersionIsRejected) {
    const auto path = write_config("model.json", R"({"device_model": "Yam"})");
    DeviceConfig config;
    EXPECT_EQ(config.init_config_model(make_arm_cla(path)), ReturnCode::NOT_FOUND);
}

TEST_F(DeviceConfigTest, GetFieldValueExtractsTypedValues) {
    DeviceConfig config;
    const json data = json::parse(R"({"name": "arm", "count": 6, "enabled": true, "scale": 0.5})");

    std::string name;
    int count = 0;
    bool enabled = false;
    float scale = 0.0f;
    EXPECT_EQ(config.get_field_value(data, "name", name), ReturnCode::SUCCESS);
    EXPECT_EQ(config.get_field_value(data, "count", count), ReturnCode::SUCCESS);
    EXPECT_EQ(config.get_field_value(data, "enabled", enabled), ReturnCode::SUCCESS);
    EXPECT_EQ(config.get_field_value(data, "scale", scale), ReturnCode::SUCCESS);
    EXPECT_EQ(name, "arm");
    EXPECT_EQ(count, 6);
    EXPECT_TRUE(enabled);
    EXPECT_FLOAT_EQ(scale, 0.5f);
}

TEST_F(DeviceConfigTest, GetFieldValueRejectsMissingField) {
    DeviceConfig config;
    const json data = json::parse(R"({"name": "arm"})");
    int count = -1;
    EXPECT_EQ(config.get_field_value(data, "count", count), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(count, -1);  // Output must be untouched on failure.
}

TEST_F(DeviceConfigTest, GetFieldValueRejectsTypeMismatch) {
    DeviceConfig config;
    const json data = json::parse(R"({"count": "six"})");
    int count = -1;
    EXPECT_EQ(config.get_field_value(data, "count", count), ReturnCode::INVALID_PARAM);
    EXPECT_EQ(count, -1);
}

TEST_F(DeviceConfigTest, EffectorOpenAtMinCascadesIntoServoDicts) {
    const auto path = write_config("effector_01.json", R"({
        "config_version": "1.1.1",
        "open_at_min": true,
        "joints": [
            {"servos": [{"servo_id": 1}, {"servo_id": 2, "open_at_min": false}]},
            {"servos": [{"servo_id": 3}]}
        ]
    })");
    CommandLineArgs cla;
    cla.device_config_type = DeviceConfigType::EFFECTOR;
    cla.effector_instance_config = path;

    DeviceConfig config;
    ASSERT_EQ(config.init_config_individual(cla), ReturnCode::SUCCESS);

    const auto& joints = config.values_["joints"];
    EXPECT_TRUE(joints[0]["servos"][0]["open_at_min"].get<bool>());
    // An explicit per-servo override must be preserved.
    EXPECT_FALSE(joints[0]["servos"][1]["open_at_min"].get<bool>());
    EXPECT_TRUE(joints[1]["servos"][0]["open_at_min"].get<bool>());
}
