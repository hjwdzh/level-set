#ifndef CONTACT_GRAPH_H_
#define CONTACT_GRAPH_H_

#include <map>
#include <list>
#include "Rigid_Geometry.h"
#include "Contact.h"
using namespace std;

class ContactGraph {
public:
    class ContactGraphNode {
    public:
        ContactGraphNode()
        : level(-1)
        {}
        Rigid_Geometry* rgd;
        list<Contact*> contacts;
        list<ContactGraphNode*> edges;
        int level;
    };
    map<Rigid_Geometry*, ContactGraphNode> graph;
    void Connect(Contact& c);
    void solveContacts();
    
private:
    vector<ContactGraphNode*> stack;
    int left, right;
    void FindStatics();
    void Expand(ContactGraphNode* node);
    bool SolveContact(ContactGraphNode* node, bool solve_nonstatic = true);
};

#endif