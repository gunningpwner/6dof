class ISensorBase {
public:
    virtual ~ISensorBase() = default;
    virtual void update(double dt) {} // For sensors with internal states/lag
};

// Templated interface for specific data types
template <typename DataT>
class ISensor : public ISensorBase {
public:
    virtual DataT read(const State& truth) = 0;
};