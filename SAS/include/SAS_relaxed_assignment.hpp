#ifndef ___SAS_RELAXED_ASSIGNMENT___
#define ___SAS_RELAXED_ASSIGNMENT___

#include <unordered_map>
#include <assert.h>
#include <iostream>
#include <stdexcept>
#include <iterator>
#include <vector>

#include "SAS_assignment.hpp"
#include "SAS_variable_info.hpp"

/*
 *
 * Class representing possible variable assignment. Pairs (variable_index, value_index) are stored
 * inside of encapsulated std::unordered_map.
 *
 */

class SAS_Relaxed_Assignment
{

public:
    SAS_Relaxed_Assignment(SAS_Assignment assignment, const std::vector<SAS_Variable_Info> varinfo);
    //SAS_Relaxed_Assignment(SAS_Relaxed_Assignment &assignment);
    //SAS_Relaxed_Assignment(const SAS_Relaxed_Assignment& rel_assignment) : _assignment(rel_assignment._assignment){};

    void set(size_t variable, size_t value);                   // set variable to given value
    const std::vector<bool> get(size_t variable) const;                         // get value of variable (throws std::out_of_range if variable not in assignment)
    bool empty() const;                                        // check if assignment is empty
    bool contains(size_t variable) const;                      // check if variable is assigned
    void apply(const SAS_Assignment &other_assignment);        // add all values of other_assignment to this assignment
    void strict_apply(const SAS_Assignment &other_assignment); // values of other_assignment become only values of affected variables of this assignment
    size_t size() const;                                       // number of assigned variables
    bool satisfy(size_t variable, size_t value) const;         // check if assignment satisty pair - return true iff variable is present in assignment and has same value
    bool satisfied_by(const SAS_Assignment &assignment) const; // check if assighment is satisfied by the other one - return true iff all pairs from this are in the other one
    bool satisfy_all(const SAS_Assignment &assignment) const;  // check if all pairs are satisfied - return true iff all pairs from the other assignment are in this one

    bool multiple_values(size_t variable) const;               // check if a variable has multiple values
    bool exist_multiple_value_variable(const SAS_Assignment &assignment) const;  // check if an assignemnt has a variable with multiple values
    bool check_applicability_with_safe(const SAS_Assignment &assignment, const std::unordered_map<size_t, std::vector<bool> > &safe);

    bool match_all(const SAS_Relaxed_Assignment &rass) const;

    void clean_duplicite_variables();

    // iterator definition
    std::unordered_map<size_t, std::vector<bool> >::const_iterator begin() const { return _assignment.cbegin(); }
    std::unordered_map<size_t, std::vector<bool> >::const_iterator end() const { return _assignment.cend(); }

    // container proxies
    //std::unordered_map<size_t, size_t>::const_iterator erase(std::unordered_map<size_t, size_t>::const_iterator it) { return _assignment.erase(it); }

private:
    std::unordered_map<size_t, std::vector<bool> > _assignment;
};

#endif
