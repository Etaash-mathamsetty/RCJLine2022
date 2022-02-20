QTRSensors::QTRSensors(const uint8_t* pins, int _length, uint8_t _emitter_pin, int _max_time = 2500);  
void QTRSensors::addOffValues(const int* off)  
int* QTRSensors::getOffValues()  
void QTRSensors::calibrate(void (*sig_func )(bool)) (function pointer: true is black calibrate false is white calibrate)  
QTRSensors::~QTRSensors()  
void QTRSensors::Update()  
uint32_t QTRSensors::get_line()  
int QTRSensors::operator[](int index)  
int QTRSensors::get_val(int index)  
