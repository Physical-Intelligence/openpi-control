/*!
 * @file pi_exception.hpp
 * @brief PiException class for custom exception handling.
 */

#pragma once
#include <exception>
#include <string>

/*!
 * @brief Custom exception class for PI Control system error handling.
 */
class PiException : public std::exception {
   public:
    /*!
     * @brief Constructor.
     * @param message Error message describing the exception.
     */
    explicit PiException(const std::string message) : message_(message) {}

    /*!
     * @brief Static helper function to throw a PiException with an error message.
     * @param message Error message describing the exception.
     */
    static void pi_error(const std::string& message) { throw PiException(message); }

    /*!
     * @brief Returns the error message as a C-style string.
     * @return Pointer to a null-terminated C-style string containing the error message.
     */
    virtual const char* what() const noexcept override { return message_.c_str(); }

   private:
    std::string message_;  ///< Internal storage for the error message.
};

