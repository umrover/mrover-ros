#pragma once

#include <NvInferRuntimeBase.h>

namespace nvinfer1 {

    class Logger final : public ILogger {
    public:
        void log(Severity severity, char const* msg) noexcept override;
    };

    template<typename T>
    struct Destroy {
        auto operator()(T* t) const -> void {
            t->destroy();
        }
    };

} // namespace nvinfer1