// Datastructures.cc
//
// Student name: EH

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
      sort(vec.begin(), vec.end(),  [](const std::pair<TownID, double> &a,
                                       const std::pair<TownID, double> &b)
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
    double dist = trunc(sqrt(pow(coord_first.x - coord_last.x,2)
                             + pow(coord_first.y - coord_last.y,2)));
    return dist;
}

std::vector<TownID> Datastructures::taxers_rekursion(TownID id,
                                                     std::vector<TownID> taxers) {
    if (towns_.find(id) == towns_.end()) {
        return taxers;
    }
    taxers.push_back(id);
    return taxers_rekursion(towns_.at(id).masterid_, taxers);
}

int Datastructures::total_net_tax_recursion(TownID id) {
        int net_tax = towns_.at(id).tax_;

        for (auto& vassal : towns_.at(id).vassals_ ) {

            net_tax += total_net_tax_recursion(vassal);
        }
        return trunc(0.1*net_tax);
}


//
// Phase 2 operations
//


void Datastructures::clear_roads()
{
    for (auto& town : towns_) {
        // Clears all neighbours
        town.second.to_neighbours_.clear();
    }
}

std::vector<std::pair<TownID, TownID>> Datastructures::all_roads()
{
    // Initializes a vector for roads and a pair for towns
    std::vector<Road> all_roads;
    Road road;

    for (auto& town : towns_) {
        for (auto& neighbour : town.second.to_neighbours_) {
            // Makes a pair of towns in alphabetical order
            if (town.first <= neighbour.first) {
                road = {town.first, neighbour.first};
            }

            else {
                road = {neighbour.first, town.first};
            }

            // Inserts road to vector if it does not exist yet
            if (std::find(all_roads.begin(), all_roads.end(), road)
                    == all_roads.end()) {
                all_roads.push_back(road);
            }
        }
    }
    return all_roads;
}

bool Datastructures::add_road(TownID town1, TownID town2)
{
    // Returns false if one of towns does not exist or there is already
    // route between them
    if (towns_.find(town1) == towns_.end() or towns_.find(town2) == towns_.end()
            or towns_.at(town1).to_neighbours_.find(town2)
            != towns_.at(town1).to_neighbours_.end()) {
        return false;
    }

    // Distance between towns
    double distance = calculate_distance(towns_.at(town1).coord_,
                                         towns_.at(town2).coord_);
    // Inserts towns to each others neighbours
    towns_.at(town1).to_neighbours_.insert({town2, distance});
    towns_.at(town2).to_neighbours_.insert({town1, distance});

    return true;
}

std::vector<TownID> Datastructures::get_roads_from(TownID id)
{
    std::vector<TownID> roads_from_id;
    //Check if id is not found
    if (towns_.find(id) == towns_.end()) {
        return {NO_TOWNID};
    }
    //Inserts all neighbours of id to vector
    for (auto& neighbour : towns_.at(id).to_neighbours_) {
        roads_from_id.push_back(neighbour.first);
    }
    return roads_from_id;
}


std::vector<TownID> Datastructures::any_route(TownID fromid, TownID toid)
{
    // Returns false if one of towns does not exist
    if (towns_.find(fromid) == towns_.end() or towns_.find(toid) == towns_.end()) {
        return {NO_TOWNID};
    }

    // Initializes route vector and drives breadth-first-search
    std::vector<TownID> route;
    bfs_for_any_route(fromid);

    // Returns empty vector if there is no route to id
    if (towns_.at(toid).distance_ == infinity) {
        return {};
    }

    // Creates route
    TownID current = toid;
    while (current != fromid) {
        route.push_back(current);
        current = towns_.at(current).prev_node_;
    }

    // Pushes last town to vector and makes reverse of it
    route.push_back(fromid);
    std::reverse(route.begin(), route.end());

    return route;
}

bool Datastructures::remove_road(TownID town1, TownID town2)
{
    // Returns false if one of towns does not exist or there is not
    // road between them
    if (towns_.find(town1) == towns_.end() or towns_.find(town2) == towns_.end()
            or towns_.at(town1).to_neighbours_.find(town2)
            == towns_.at(town1).to_neighbours_.end()) {
        return false;
    }

    // Removes towns from their neighbours
    towns_.at(town1).to_neighbours_.erase(town2);
    towns_.at(town2).to_neighbours_.erase(town1);
    return true;
}

std::vector<TownID> Datastructures::least_towns_route(TownID fromid, TownID toid)
{
    // Calls any_route because it already finds the shortest route
    return any_route(fromid, toid);
}

std::vector<TownID> Datastructures::road_cycle_route(TownID startid)
{
    // Checks if town is not found
    if (towns_.find(startid) == towns_.end()) {
        return {NO_TOWNID};
    }

    // Figures out loop with death-first-search algorithm and returns empty
    // list if there is no loop. First element in pair is town where loop
    // comes and second element is the previous town before loop
    Road loop = dfs_for_road_cycle_roat(startid);
    if (loop.first == "") {
        return {};
    }

    // Figures out loop path from last town to starting town
    std::vector<TownID> cycle_route;
    cycle_route.push_back(loop.first);
    TownID current = loop.second;
    while (current != startid) {
        cycle_route.push_back(current);
        current = towns_.at(current).prev_node_;
    }

    // Inserts the starting townt to vector and makes reverse transform to it
    cycle_route.push_back(startid);
    std::reverse(cycle_route.begin(), cycle_route.end());

    return cycle_route;

}

std::vector<TownID> Datastructures::shortest_route(TownID fromid, TownID toid)
{
    // Returns false if one of towns does not exist
    if (towns_.find(fromid) == towns_.end() or towns_.find(toid) == towns_.end()) {
        return {NO_TOWNID};
    }

    // Initializes route vector and drives breadth-first-search
    std::vector<TownID> route;
    dijkstra_for_shortest_route(fromid);

    // Returns empty vector if there is no route to id
    if (towns_.at(toid).distance_ == infinity) {
        return {};
    }

    // Creates route
    TownID current = toid;
    while (current != fromid) {
        route.push_back(current);
        current = towns_.at(current).prev_node_;
    }

    // Pushes last town to vector and makes reverse of it
    route.push_back(fromid);
    std::reverse(route.begin(), route.end());

    return route;
}

Distance Datastructures::trim_road_network()
{

    // Makes a vector of all roads
    std::vector<Road> roads = all_roads();

    // Initializes vector for roads with their weights
    std::vector<std::pair<Distance, Road>> roads_with_weights;

    Distance MST_weight = 0;

    // Calculates weights of roads to vector
    for (auto& road : roads) {
        Distance dist = calculate_distance(towns_.at(road.first).coord_,
                                           towns_.at(road.second).coord_);
        roads_with_weights.push_back({dist, road});
    }

    // Initalizes each parent and rank
    for (auto& town : towns_) {
        town.second.parent_ = town.first;
        town.second.rank_ = 0;
    }

    // Sorts roads based on their weights in increasing order
        std::sort(roads_with_weights.begin(), roads_with_weights.end(),
                  [](const std::pair<Distance, Road> road1,
                     const std::pair<Distance, Road> road2)
        {return road1.first < road2.first;});


    // Apply union-by-rank technique to find the minimum spanning tree and
    // calculates the final lenght of network
    for (auto& road : roads_with_weights) {
        TownID root1 = find_root(road.second.first);
        TownID root2 = find_root(road.second.second);

        if (root1 != root2) {
            MST_weight += road.first;
            if (towns_.at(root1).rank_ < towns_.at(root2).rank_) {
                towns_.at(root1).parent_ = root2;
                towns_.at(root2).rank_ ++;
            } else {
                towns_.at(root2).parent_ = root1;
                towns_.at(root1).rank_ ++;
            }
        } else {
            // Removing all roads that does not include
            remove_road(road.second.first, road.second.second);
        }
    }

    return MST_weight;
}

// Own functions

TownID Datastructures::find_root(TownID town) {
    if (town != towns_.at(town).parent_) {
        towns_.at(town).parent_ = find_root(towns_.at(town).parent_);
    }

    return towns_.at(town).parent_;
}

// Drives breadth-first-search algorithm for searching any route from
// starting town to all other towns in road network
// Takes town id as a parameter and returns nothing
void Datastructures::bfs_for_any_route(TownID id) {

   // Initializes search variables
    for (auto& town : towns_) {
        town.second.colour = WHITE;
        town.second.distance_ = infinity;
        town.second.prev_node_ = "";
    }

    // Initializes list and starts bfs
    std::deque<TownID> deque;
    towns_.at(id).colour = GRAY;
    towns_.at(id).distance_ = 0;
    deque.push_back(id);

    while (!deque.empty()) {
        TownID last = deque.front();
        deque.pop_front();
        for (auto& neighbour : towns_.at(last).to_neighbours_) {
            if (towns_.at(neighbour.first).colour == WHITE) {
                towns_.at(neighbour.first).colour = GRAY;
                towns_.at(neighbour.first).distance_ = towns_.at(last).distance_ +
                        calculate_distance(towns_.at(neighbour.first).coord_,
                                           towns_.at(last).coord_);
                towns_.at(neighbour.first).prev_node_ = last;
                deque.push_back(neighbour.first);
            }
        } towns_.at(last).colour = BLACK;
    }

}

// Drives depth-first-search algorithm for searching any cycle from starting town
// Takes town id as a parameter and returns pair of loop town and one before it
Road Datastructures::dfs_for_road_cycle_roat(TownID id) {

    std::pair <TownID, TownID> loop = {};
   // Initializes search variables
    for (auto& town : towns_) {
        town.second.colour = WHITE;
        town.second.prev_node_ = "";
    }

    // Initializes list and starts dfs
    std::stack<TownID> stack;
    stack.push(id);

    while (!stack.empty()) {
        TownID top = stack.top();
        stack.pop();
        if (towns_.at(top).colour == WHITE) {
            towns_.at(top).colour = GRAY;
            stack.push(top);
            for (auto& neighbour : towns_.at(top).to_neighbours_) {
                if (towns_.at(neighbour.first).colour == WHITE) {
                    stack.push(neighbour.first);
                    towns_.at(neighbour.first).prev_node_ = top;
                }
                if (towns_.at(neighbour.first).colour == GRAY and
                        neighbour.first != towns_.at(top).prev_node_){
                    loop.first = neighbour.first;
                    loop.second = top;
                    return loop;
                }
            }
        } else {
            towns_.at(top).colour = BLACK;
        }
    } return loop;

}

// Drives Dijkstra algorithm for searching the shortest routes from starting town to all
// other towns in road network
// Takes town id as a parameter and returns nothing
void Datastructures::dijkstra_for_shortest_route(TownID id) {

   // Initializes search variables
    for (auto& town : towns_) {
        town.second.colour = WHITE;
        town.second.distance_ = infinity;
        town.second.prev_node_ = "";
    }

    // Initializes priority queue and starts bfs
    std::priority_queue<std::pair<int, TownID>,
            std::vector<std::pair<int, TownID>>,
            std::greater<std::pair<int, TownID>>> deque;

    towns_.at(id).colour = GRAY;
    towns_.at(id).distance_ = 0;
    deque.push({towns_.at(id).distance_ , id});


    while (!deque.empty()) {
        std::pair<double, TownID> nearest_town = deque.top();
        deque.pop();
        if (towns_.at(nearest_town.second).distance_ < nearest_town.first) {
            continue;
        }

        for (auto& neighbour : towns_.at(nearest_town.second).to_neighbours_) {
            bool is_shorter = false;
            if (towns_.at(neighbour.first).distance_ >
                    towns_.at(nearest_town.second).distance_
                    + calculate_distance(towns_.at(neighbour.first).coord_,
                                         towns_.at(nearest_town.second).coord_)){

                // Updates distance if new one is shorter than earlier one
                towns_.at(neighbour.first).distance_ =
                        towns_.at(nearest_town.second).distance_
                        + calculate_distance(towns_.at(neighbour.first).coord_,
                                             towns_.at(nearest_town.second).coord_);
                // Updates previous town
                towns_.at(neighbour.first).prev_node_ = nearest_town.second;
                is_shorter = true;
            }

            if (towns_.at(neighbour.first).colour == WHITE) {
                towns_.at(neighbour.first).colour = GRAY;
                deque.push({towns_.at(neighbour.first).distance_, neighbour.first});
            }

            else {
                if (is_shorter) {
                    deque.push({towns_.at(neighbour.first).distance_, neighbour.first});
                }
            }
        } towns_.at(nearest_town.second).colour = BLACK;
    }
}
