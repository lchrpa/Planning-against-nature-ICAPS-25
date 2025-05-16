#include "SAS_operator.hpp"

const SAS_Assignment SAS_Operator::get_grounded_preconditions(const SAS_State &state) const
{
    SAS_Assignment grounded_preconditions(_preconditions);
    for (auto conditional_effect : _effects)
    {
        if (state.satisfy_all(conditional_effect.first))
        {
            for (auto precondition : conditional_effect.first)
            {
                grounded_preconditions.set(precondition.first, precondition.second);
            }
        }
    }
    return grounded_preconditions;
}

const SAS_Assignment SAS_Operator::get_grounded_effects(const SAS_State &state) const
{
    SAS_Assignment grounded_effects;
    for (auto conditional_effect : _effects)
    {
        if (state.satisfy_all(conditional_effect.first))
        {
            grounded_effects.apply(conditional_effect.second);
        }
    }
    return grounded_effects;
}

const SAS_Assignment SAS_Operator::get_minimal_effects() const
{
    SAS_Assignment minimal_effects;
    for (auto conditional_effect : _effects)
    {
        if (conditional_effect.first.empty())
        {
            minimal_effects.apply(conditional_effect.second);
        }
    }
    return minimal_effects;
}

const std::string SAS_Operator::strips_name_to_sas(const std::string &strips_name)
{
    if (strips_name == "<noop-action>")
    {
        return strips_name;
    }
    else
    {
        return strips_name.substr(1, strips_name.size() - 1);
    }
}

const std::set<size_t> SAS_Operator::get_variables() const
{
std::set<size_t> variables;

for (auto prec : _preconditions){

    variables.insert(prec.first);
}

for (auto effs : _effects){

    for (auto cond : effs.first)
        variables.insert(cond.first);

    for (auto condeff : effs.second)
        variables.insert(condeff.first);

}

return variables;
}

bool SAS_Operator::isAchieverFor(const SAS_Operator& op)
{
  const SAS_Assignment effs=get_minimal_effects();
  const SAS_Assignment prec=op.get_preconditions();

for (auto eff : effs){
    if (prec.contains(eff.first) && prec.get(eff.first)==eff.second) return true;
    }
    return false;
}

bool SAS_Operator::isClobbererFor(const SAS_Operator& op)
{
  const SAS_Assignment effs=get_minimal_effects();
  const SAS_Assignment prec=op.get_preconditions();

for (auto eff : effs){
    if (prec.contains(eff.first) && prec.get(eff.first)!=eff.second) return true;
    }
    return false;
}

const SAS_Assignment SAS_Operator::hold_after(bool with_waitfor)
{
    SAS_Assignment ha(get_minimal_effects());
    for (auto pre : get_preconditions()){
        if (!ha.contains(pre.first)) ha.set(pre.first,pre.second);
    }
    if (with_waitfor)
      for (auto pre : get_waitfor()){
        if (!ha.contains(pre.first)) ha.set(pre.first,pre.second);
       }
    return ha;
}

bool SAS_Operator::deletesFact(size_t var, size_t val)
{
    if (!get_minimal_effects().contains(var)||get_minimal_effects().get(var)==val) return false;
    if (get_preconditions().contains(var) && get_preconditions().get(var)!=val) return false;

    return true;
}

void SAS_Operator::to_sas_file(std::ofstream& file)
{
    SAS_Assignment effs = get_minimal_effects();

    SAS_Assignment prevail;

    for (auto x : _preconditions){
        if (!effs.contains(x.first)) prevail.set(x.first,x.second);
    }

    file << "begin_operator" << std::endl;
    file << _name << std::endl;
    prevail.to_sas_file(file);
    file << effs.size() << std::endl;

    for (auto e : effs){
        file << "0 "  << e.first << " ";
        if (_preconditions.contains(e.first)) file << _preconditions.get(e.first); else file << "-1";
        file << " " << e.second << std::endl;
    }
    file << "1" << std::endl;

    file << "end_operator" << std::endl;
}
