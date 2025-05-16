#ifndef ___SAS_OPERATOR___
#define ___SAS_OPERATOR___

#include <utility>
#include <vector>
#include <set>
#include <string>
#include <functional>

class SAS_State;
class SAS_Action;

#include "SAS_assignment.hpp"
#include "SAS_state.hpp"

class SAS_Operator
{

public:
    SAS_Operator(std::string name, SAS_Assignment preconditions, std::vector<std::pair<SAS_Assignment, SAS_Assignment>> effects, unsigned int cost) : _name(name), _preconditions(preconditions), _effects(effects), _cost(cost) {}
    inline const std::vector<std::pair<SAS_Assignment, SAS_Assignment>> &get_effects() const { return _effects; }
    const SAS_Assignment get_grounded_effects(const SAS_State &state) const;
    const SAS_Assignment get_minimal_effects() const;
    inline const SAS_Assignment &get_preconditions() const { return _preconditions; }
    const SAS_Assignment get_grounded_preconditions(const SAS_State &state) const;
    inline const std::string get_name() const { return _name; }
    inline unsigned int get_cost() const { return _cost; }

    //The following methods do not support conditional effects
    bool isAchieverFor(const SAS_Operator &op);
    bool isClobbererFor(const SAS_Operator &op);
    const SAS_Assignment hold_after(bool with_waitfor = false);

    bool deletesFact(size_t var,size_t val);


    static const std::string strips_name_to_sas(const std::string &strips_name);

    const std::set<size_t> get_variables() const;

    inline virtual bool isEvent(){return false;}

    void to_sas_file(std::ofstream &file);

    inline const SAS_Assignment &get_waitfor() const { return _waitfor; }
    inline void set_waitfor(SAS_Assignment waitfor) {_waitfor=waitfor;}

protected:
    std::string _name;
    SAS_Assignment _preconditions;
    std::vector<std::pair<SAS_Assignment, SAS_Assignment>> _effects;
    unsigned int _cost;

    SAS_Assignment _waitfor;
};

#endif
