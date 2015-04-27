#include "ContactGraph.h"

void ContactGraph::Connect(Contact& c) {
    map<Rigid_Geometry*, ContactGraphNode>::iterator it1, it2;
    it1 = graph.find(c.a);
    if (it1 == graph.end()) {
        graph.insert(make_pair(c.a, ContactGraphNode()));
        it1 = graph.find(c.a);
    }
    it2 = graph.find(c.b);
    if (it2 == graph.end()) {
        graph.insert(make_pair(c.b, ContactGraphNode()));
        it2 = graph.find(c.b);
    }
    
    it1->second.rgd = c.a;
    it1->second.contacts.push_back(&c);
    it1->second.edges.push_back(&(it2->second));
    
    it2->second.rgd = c.b;
    it2->second.contacts.push_back(&c);
    it2->second.edges.push_back(&(it1->second));
}

void ContactGraph::solveContacts() {
    FindStatics();
    int max_iteration = 3;
    while (left < right) {
        int current = right;
        for (int i = left; i < current; ++i) {
            Expand(stack[i]);
        }
        bool has_collision = true;
        int iteration = 0;
        while (has_collision) {
            iteration++;
            has_collision = false;
            for (int i = current; i < right; ++i) {
                if (SolveContact(stack[i], iteration < max_iteration)) {
                    has_collision = true;
                }
            }
            if (iteration > max_iteration)
                break;
        }
        left = current;
    }
}

void ContactGraph::FindStatics() {
    stack.resize(graph.size());
    left = 0, right = 0;
    map<Rigid_Geometry*, ContactGraphNode>::iterator it;
    for (it = graph.begin(); it != graph.end(); ++it) {
        if (it->first->nailed) {
            it->second.level = 0;
            stack[right++] = &(it->second);
        }
    }
}

void ContactGraph::Expand(ContactGraphNode* node) {
    for (list<ContactGraphNode*>::iterator it = node->edges.begin();
         it != node->edges.end(); ++it) {
        if ((*it)->level == -1) {
            (*it)->level = node->level + 1;
            stack[right++] = (*it);
        }
    }
}

bool ContactGraph::SolveContact(ContactGraphNode* node, bool solve_nonstatic) {
    list<Contact*>::iterator it1 = node->contacts.begin();
    bool res = false;
        for (list<ContactGraphNode*>::iterator it = node->edges.begin();
             it != node->edges.end(); ++it, ++it1) {
            if (solve_nonstatic && (*it)->level == node->level) {
                res |= (*it1)->collide_handling(0);
            }
            if ((*it)->level != -1 && (*it)->level < node->level) {
                (*it)->rgd->nailed = true;
                res |= (*it1)->collide_handling(0);
                if ((*it)->level != 0) {
                    (*it)->rgd->nailed = false;
                }
            }
        }
    return res;
}

