
class Observer {
  public:
    Observer(void (*fn)());
    int8_t doCode(uint32_t interval, uint32_t execTime);
  private:
    uint32_t lastTime;
    void (*methodToExecute)();
};
