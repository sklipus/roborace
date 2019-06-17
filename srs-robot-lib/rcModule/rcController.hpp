#ifndef rcController_hpp
#define rcController_hpp

class rcController {
    public:
        void init(int pin1, int pin2);
        void scan();
        int getCh1();
        int getCh2();
        int isRcEnabled();
    private:
        int ch1;
        int ch2;
        int pin1;
        int pin2;
        int isEnabled;
};

#endif