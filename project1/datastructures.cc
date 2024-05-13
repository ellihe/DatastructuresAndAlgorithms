// Datastructures.cc
//
// Creator name: Ellinoora Hetemaa

#include "datastructures.hh"

#include <random>

#include <cmath>

std::minstd_rand rand_engine; // Reasonably quick pseudo-random generator

template <typename Type>
Type random_in_range(Type start, Type end)
{
    auto range = end-start;
    ++range;

    auto num = std::uniform_int_distribution<unsigned long int>(0, range-1)(rand_engine);

    return static_cast<Type>(start+num);
}

// Modify the code below to implement the functionality of the class.
// Also remove comments from the parameter names when you implement
// an operation (Commenting out parameter name prevents compiler from
// warning about unused parameters on operations you haven't yet implemented.)

Datastructures::Datastructures()
{
    // Write any initialization you need here
}

Datastructures::~Datastructures()
{
    // Write any cleanup you need here
}

unsigned int Datastructures::town_count()
{
    int number_of_towns = towns_.size();
    return number_of_towns;
}


void Datastructures::clear_all()
{
    //Removes all towns from datastructure
    towns_.clear();
}

bool Datastructures::add_town(TownID id, const Name &name, Coord coord, int tax)
{

    if (towns_.find(id) == towns_.end()) {
        //Initializes new town if and only if there is
        //not town with same id in datastructure
        Town new_town;
        new_town.id_ = id;
        new_town.name_ = name;
        new_town.coord_ = coord;
        new_town.tax_ = tax;
        //Inserts town to datastructure
        towns_.insert({id, new_town});
        return true;
    } else {
        return false;
    }
}

Name Datastructures::get_town_name(TownID id)
{
    //Checks if id is found or not
    if (towns_.find(id) == towns_.end()) {
        return NO_NAME;
    }

    return towns_.at(id).name_;

}

Coord Datastructures::get_town_coordinates(TownID id)
{
    //Checks if id is not found
    if (towns_.find(id) == towns_.end()) {
        return NO_COORD;
    }

    return towns_.at(id).coord_;
}

int Datastructures::get_town_tax(TownID id)
{
    //Checks if id is not found
    if (towns_.find(id) == towns_.end()) {
        return NO_VALUE;
    }

    return towns_.at(id).tax_;
}

std::vector<TownID> Datastructures::all_towns()
{
    //Initializes vector for ids
    std::vector<TownID> towns;
    //Takes all ids from datastructure
    for (auto& town :towns_) {
        towns.push_back(town.first);
    }
    return towns;
}

std::vector<TownID> Datastructures::find_towns(const Name &name)
{
    //Initializes vector for ids
    std::vector<TownID> towns;
    //Looks for all towns with certain name and
    //inserts their ids to vector
    for (auto& town :towns_) {
        if (town.second.name_ == name)
            towns.push_back(town.first);
    }
    return towns;
}

bool Datastructures::change_town_name(TownID id, const Name &newname)
{
    //Checks if id is not found
    if (towns_.find(id) == towns_.end()) {
        return false;
    }

    towns_.at(id).name_ = newname;
    return true;
}

std::vector<TownID> Datastructures::towns_alphabetically()
{
    //Initialize vectors and name
    std::vector<std::pair<TownID, Name>> vec;
    std::vector <TownID> alphabetical_towns;
    Name name;

    //Copy key-value pairs from the map to the vector
    std::unordered_map<TownID, Town> :: iterator it2;
    for (it2=towns_.begin(); it2!=towns_.end(); it2++)
    {
        name = it2->second.name_;
        vec.push_back(make_pair(it2->first,name));
    }

    //Sort the vector by increasing order of names
    sort(vec.begin(), vec.end(),  [](const std::pair<TownID, Name> &a, const std::pair<TownID, Name> &b)
    {return a.second < b.second;});

    //Insert all ids from vector pairs to vector of ids
    for (auto& town : vec) {
      alphabetical_towns.push_back(town.first);
    }
    return alphabetical_towns;

}

std::vector<TownID> Datastructures::towns_distance_increasing()
{
    //Initialize origo's coordinates
    Coord origo = {0,0};
    return towns_nearest(origo);
}

TownID Datastructures::min_distance()
{
    //Initialize vector with all distances in increasing order
    std::vector<TownID> distances = towns_distance_increasing();
    //Check if vector is empty
    if (distances.size() == 0) {
        return NO_TOWNID;
    }

    return distances.front();
}

TownID Datastructures::max_distance()
{
    //Initialize vector with all distances in increasing order
    std::vector<TownID> distances = towns_distance_increasing();
    //Check if vector is empty
    if (distances.size() == 0) {
        return NO_TOWNID;
    }
    return distances.back();
}

bool Datastructures::add_vassalship(TownID vassalid, TownID masterid)
{
    //Check if vassalid id or master id is not found
    if (towns_.find(vassalid) == towns_.end() or towns_.find(masterid) == towns_.end()) {
        return false;
    }

    //Check if vassalid already has a master
    if (towns_.find(towns_.at(vassalid).masterid_) != towns_.end()) {
        return false;
    }
    //Update datastructure
    towns_.at(vassalid).masterid_ = masterid;
    towns_.at(masterid).vassals_.push_back(vassalid);
    return true;
}

std::vector<TownID> Datastructures::get_town_vassals(TownID id)
{

    //Check if id is not found
    if (towns_.find(id) == towns_.end()) {
        return {NO_TOWNID};
    }
    //Inserts all vassals of id to vector
    return towns_.at(id).vassals_;
}


std::vector<TownID> Datastructures::taxer_path(TownID id)
{
    //Initialize id vector
    std::vector<TownID> taxers;
    // Call recursive function
    return taxers_rekursion(id, taxers);
}

bool Datastructures::remove_town(TownID id)
{
    // Check if id is not found
    if (towns_.find(id) == towns_.end()) {
        return false;
    }
    // Check id id has master
    if (towns_.find(towns_.at(id).masterid_) != towns_.end() ) {
        // Update all id's vassals to be id's master's vassals
        for (auto& vassal : towns_.at(id).vassals_) {
            towns_.at(vassal).masterid_ = towns_.at(id).masterid_;
            towns_.at(towns_.at(id).masterid_).vassals_.push_back(vassal);
        }
        // Removes id from id's master's vassals
        towns_.at(towns_.at(id).masterid_).vassals_
                 .erase (std::remove(towns_.at(towns_.at(id).masterid_)
                 .vassals_.begin(), towns_.at(towns_.at(id).masterid_)
                 .vassals_.end(), id), towns_.at(towns_.at(id).masterid_)
                 .vassals_.end());
        // Removes id from datastructure
        towns_.erase(id);
        return true;

    }
    // if id does not have master, then change all vassal's master to empty
    TownID new_masterid;
    for (auto& vassal : towns_.at(id).vassals_) {
        towns_.at(vassal).masterid_ = new_masterid;
    }
    // Removes id from datastructure
    towns_.erase(id);
    return true;
}

std::vector<TownID> Datastructures::towns_nearest(Coord coord)
{
    std::vector<std::pair<TownID, double>> vec;
    std::vector <TownID> distances;
    double distance;

    //Copy key-value pairs from the map to the vector
    std::unordered_map<TownID, Town> :: iterator it2;
    for (it2=towns_.begin(); it2!=towns_.end(); it2++)
    {
        distance = calculate_distance(it2->second.coord_, coord);
        vec.push_back(make_pair(it2->first, distance));
    }

      //Sort the vector by increasing order of distance
      sort(vec.begin(), vec.end(),  [](const std::pair<TownID, double> &a, const std::pair<TownID, double> &b)
      {return a.second < b.second;});

      for (auto& town : vec) {
          distances.push_back(town.first);
      }
      return distances;

    }

std::vector<TownID> Datastructures::longest_vassal_path(TownID id)
{
    // Check if id is not found
    if (towns_.find(id) == towns_.end()) {
        return {NO_TOWNID};
    }

    //Initialize vectors
    std::vector<TownID> longest_path;
    std::vector<TownID> path;

    // Make clear the longest path with recursion
    for (auto& vassal : towns_.at(id).vassals_ ) {
        path = longest_vassal_path(vassal);
        if (path.size() > longest_path.size()) {
            longest_path = path;
            longest_path = path;
        }
    }

    // Insert masterid to begin of vector
    longest_path.insert(longest_path.begin(), id);
    return longest_path;

}


int Datastructures::total_net_tax(TownID id)
{

    // Check if id is not found
    if (towns_.find(id) == towns_.end()) {
        return NO_VALUE;
    }

    // Initialize current net_tax
    int net_tax = towns_.at(id).tax_;

    // Figure out all sum of all net_taxes by recursion
    for (auto& vassal : towns_.at(id).vassals_ ) {
        net_tax += total_net_tax_recursion(vassal);
    }

    // Check if id has no master
    if (towns_.find(towns_.at(id).masterid_) == towns_.end()) {
        return net_tax;
    }

    // If id has master reduce 10 % from net_tax
    return net_tax - trunc(net_tax*0.1);

}

double Datastructures::calculate_distance(Coord coord_first, Coord coord_last) {
    double dist = sqrt(pow(coord_first.x - coord_last.x,2) + pow(coord_first.y - coord_last.y,2));
    return dist;
}

std::vector<TownID> Datastructures::taxers_rekursion(TownID id, std::vector<TownID> taxers) {
    if (towns_.find(id) == towns_.end()) {
        return taxers;
    }
    taxers.push_back(id);
    return taxers_rekursion(towns_.at(id).masterid_, taxers); //will be called N times
}

//int Datastructures::total_net_tax_recursion(TownID id, int tax) {



int Datastructures::total_net_tax_recursion(TownID id) {
        int net_tax = towns_.at(id).tax_;

        for (auto& vassal : towns_.at(id).vassals_ ) {

            net_tax += total_net_tax_recursion(vassal);
        }
        return trunc(0.1*net_tax);
}

