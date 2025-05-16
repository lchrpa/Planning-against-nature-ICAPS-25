#ifndef ___SAS_ASSIGNMENT___
#define ___SAS_ASSIGNMENT___

#include <unordered_map>
#include <assert.h>
#include <iostream>
#include <stdexcept>
#include <iterator>

#include<fstream>

/*
 *
 * Class representing possible variable assignment. Pairs (variable_index, value_index) are stored 
 * inside of encapsulated std::unordered_map.
 * 
 */

class SAS_Assignment
{

public:
    SAS_Assignment(std::unordered_map<size_t, size_t> assignment) : _assignment(assignment){};
    SAS_Assignment(size_t variable, size_t value);
    SAS_Assignment() : _assignment(std::unordered_map<size_t, size_t>()){};

    void set(size_t variable, size_t value);                   // set variable to given value
    size_t get(size_t variable) const;                         // get value of variable (throws std::out_of_range if variable not in assignment)
    bool empty() const;                                        // check if assignment is empty
    bool contains(size_t variable) const;                      // check if variable is assigned
    void apply(const SAS_Assignment &other_assignment);        // set all pairs from other_assignment to this assignment
    size_t size() const;                                       // number of assigned variables
    bool satisfy(size_t variable, size_t value) const;         // check if assignment satisty pair - return true iff variable is present in assignment and has same value
    bool satisfied_by(const SAS_Assignment &assignment) const; // check if assighment is satisfied by the other one - return true iff all pairs from this are in the other one
    bool satisfy_all(const SAS_Assignment &assignment) const;  // check if all pairs are satisfied - return true iff all pairs from the other assignment are in this one

    bool any_mismatch(const SAS_Assignment &assignment) const; //check if some varaible assignment differs
    SAS_Assignment get_mismatch(const SAS_Assignment &assignment) const; //list variables and their values that differ

    // iterator definition
    std::unordered_map<size_t, size_t>::const_iterator begin() const { return _assignment.cbegin(); }
    std::unordered_map<size_t, size_t>::const_iterator end() const { return _assignment.cend(); }

    // container proxies
    std::unordered_map<size_t, size_t>::const_iterator erase(std::unordered_map<size_t, size_t>::const_iterator it) { return _assignment.erase(it); }
    size_t erase(size_t variable){return _assignment.erase(variable);}

    void to_sas_file(std::ofstream &file) const;

private:
    std::unordered_map<size_t, size_t> _assignment;
};

#endif
