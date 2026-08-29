#pragma  once

#include <cstdint>

// contants
#define PI 3.14159265359f


using ret_val = enum : std::uint8_t{
    OK = 0 ,
    ERROR
};

struct trans_data_t{
    uint8_t* data_ptr;
    uint8_t len;
};

//types
struct alpha_beta_t
{
    float alpha;
    float beta;
};
struct d_q_t
{
    float d;
    float q;
};
struct abc_phase_t
{
    float a;
    float b;
    float c;

    // for readiness and vector
    abc_phase_t operator+(float s);
    abc_phase_t operator+(abc_phase_t s);
    abc_phase_t operator*(float s);
    abc_phase_t operator*(abc_phase_t s);

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

