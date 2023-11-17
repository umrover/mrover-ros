#include "ioHelper.cuh"

using namespace std;

namespace nvinfer1 {

    string getBasename(string const& path) {
#ifdef _WIN32
        constexpr char SEPARATOR = '\\';
#else
        constexpr char SEPARATOR = '/';
#endif
        int baseId = path.rfind(SEPARATOR) + 1;
        return path.substr(baseId, path.rfind('.') - baseId);
    }

    ostream& operator<<(ostream& o, const ILogger::Severity severity) {
        switch (severity) {
            case ILogger::Severity::kINTERNAL_ERROR:
                o << "INTERNAL_ERROR";
                break;
            case ILogger::Severity::kERROR:
                o << "ERROR";
                break;
            case ILogger::Severity::kWARNING:
                o << "WARNING";
                break;
            case ILogger::Severity::kINFO:
                o << "INFO";
                break;
            default:
                break;
        }
        return o;
    }

    void writeBuffer(void* buffer, size_t size, string const& path) {
        ofstream stream(path.c_str(), ios::binary);

        if (stream)
            stream.write(static_cast<char*>(buffer), size);
    }

    // Returns empty string iff can't read the file
    string readBuffer(string const& path) {
        string buffer;
        ifstream stream(path.c_str(), ios::binary);

        if (stream) {
            stream >> noskipws;
            copy(istream_iterator<char>(stream), istream_iterator<char>(), back_inserter(buffer));
        }

        return buffer;
    }

} // namespace nvinfer1