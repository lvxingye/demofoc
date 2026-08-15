#include <cstdint>

typedef uint8_t ret_val;

class logging{
public:
    logging();
    ~logging();

    enum LOGGING_LEVEL{
        INFO=0,

    };

    ret_val log(const char* tag,LOGGING_LEVEL level, const char* message);
};