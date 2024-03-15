namespace mrover {
class I2CRuntimeError : public std::runtime_error {
public:
    explicit I2CRuntimeError(const std::string& message) : std::runtime_error(message) {}
};


}
