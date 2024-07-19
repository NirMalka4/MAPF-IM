#include "IConflict.h"

IConflict::IConflict(const std::vector<int>& agents_indices, const int timestep): agents_indices(agents_indices), timestep(timestep){}

std::ostream& operator << (std::ostream& out, const IConflict* conflict)
{
    return out << conflict->ToString();
}