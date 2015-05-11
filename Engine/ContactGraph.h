#ifndef CONTACT_GRAPH_H_
#define CONTACT_GRAPH_H_

#include <map>
#include <list>
#include <vector>
#include "Rigid_Geometry.h"
#include "Contact.h"
using namespace std;

class ContactGraph {
public:
    class ContactGraphNode {
    public:
        ContactGraphNode()
        : dfn(-1), instack(0), inComponent(-1)
        {}
        Rigid_Geometry* rgd;
        list<Contact*> contacts;
        list<ContactGraphNode*> edges;
        list<ContactGraphNode*> bedges;
        int low, dfn, instack, inComponent;
    };
    map<Rigid_Geometry*, ContactGraphNode> graph;
    void Connect(Contact& c);
    void solveContacts();
    
private:
    vector<ContactGraphNode*> stack;
    vector<int> comp_stack, level, degree;
    vector<vector<ContactGraphNode*> > components;
    int index;
    void BuildLevels();
    void tarjan(ContactGraphNode*);
    void BuildComponents();
    void SolveContact(int component);
    bool SolveContact(ContactGraphNode* node, bool solve_nonstatic = true);
};

#endif