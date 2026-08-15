#include <cstdint>

typedef uint8_t ret_val;

struct trans_data_t{
    uint8_t* data_ptr;
    uint8_t len;
};


class logging{
public:
    logging();
    ~logging();

    enum LOGGING_LEVEL{
        INFO=0,

    };

    ret_val log(const char* tag,LOGGING_LEVEL level, const char* message);
};

