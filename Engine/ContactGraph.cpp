#include "ContactGraph.h"

void ContactGraph::Connect(Contact& c) {
    map<Rigid_Geometry*, ContactGraphNode>::iterator it1, it2;
    it1 = graph.find(c.a);
    if (it1 == graph.end()) {
        graph.insert(make_pair(c.a, ContactGraphNode()));
        it1 = graph.find(c.a);
        it1->second.rgd = c.a;
    }
    it2 = graph.find(c.b);
    if (it2 == graph.end()) {
        graph.insert(make_pair(c.b, ContactGraphNode()));
        it2 = graph.find(c.b);
        it2->second.rgd = c.b;
    }
    
    if ((c.n[1] < 0 && c.b->nailed == false) || c.a->nailed) {
        it2->second.contacts.push_back(&c);
        it1->second.edges.push_back(&(it2->second));
        it2->second.bedges.push_back(&(it1->second));
    } else {
        it1->second.contacts.push_back(&c);
        it2->second.edges.push_back(&(it1->second));
        it1->second.bedges.push_back(&(it2->second));
    }
}

void ContactGraph::solveContacts() {
    BuildComponents();
    BuildLevels();
    int l = 0;
    while (l < comp_stack.size()) {
        int i = comp_stack[l];
        if (level[i] > 0) {
            SolveContact(i);
        }
        for (int j = 0; j < components[i].size(); ++j) {
            for (list<ContactGraphNode*>::iterator it = components[i][j]->edges.begin();
                 it != components[i][j]->edges.end(); ++it) {
                if ((*it)->inComponent != i) {
                    degree[(*it)->inComponent]--;
                    if (degree[(*it)->inComponent] == 0) {
                        comp_stack.push_back((*it)->inComponent);
                        level[(*it)->inComponent] = level[i] + 1;
                    }
                }
            }
        }
        l++;
    }
}

void ContactGraph::BuildLevels() {
    map<Rigid_Geometry*, ContactGraphNode>::iterator it;
    for (int i = 0; i < components.size(); ++i) {
        for (int j = 0; j < components[i].size(); ++j) {
            for (list<ContactGraphNode*>::iterator it = components[i][j]->edges.begin();
                 it != components[i][j]->edges.end(); ++it) {
                if ((*it)->inComponent != i) {
                    degree[(*it)->inComponent]++;
                }
            }
        }
    }
    comp_stack.clear();
    for (it = graph.begin(); it != graph.end(); ++it) {
        if (it->first->nailed) {
            if (level[it->second.inComponent] == -1) {
                level[it->second.inComponent] = 0;
                degree[it->second.inComponent] = 0;
            }
        }
    }
    for (int i = 0; i < components.size(); ++i) {
        if (degree[i] == 0) {
            comp_stack.push_back(i);
        }
    }
}

void ContactGraph::SolveContact(int component) {
    bool has_collision = true;
    int iteration = 0;
    int max_iteration = 3;
    while (has_collision) {
        if (iteration > max_iteration)
            break;
        has_collision = false;
        for (vector<ContactGraphNode*>::iterator it = components[component].begin();
             it != components[component].end(); ++it) {
            if (SolveContact(*it, iteration < max_iteration)) {
                has_collision = true;
            }
        }
        iteration++;
    }
}

bool ContactGraph::SolveContact(ContactGraphNode* node, bool solve_nonstatic) {
    list<Contact*>::iterator it1 = node->contacts.begin();
    bool res = false;
    if (solve_nonstatic) {
        for (list<ContactGraphNode*>::iterator it = node->bedges.begin();
             it != node->bedges.end(); ++it, ++it1) {
            if (level[(*it)->inComponent] == level[node->inComponent]) {
                res |= (*it1)->collide_handling(0);
            }
            if (level[(*it)->inComponent] != -1 && level[(*it)->inComponent] < level[node->inComponent]) {
                bool nailed = (*it)->rgd->nailed;
                (*it)->rgd->nailed = true;
                Matrix3d jr = (*it)->rgd->Jr;
                (*it)->rgd->Jr = Matrix3d::createScale(0, 0, 0);
                res |= (*it1)->collide_handling(0);
                (*it)->rgd->nailed = nailed;
                (*it)->rgd->Jr = jr;
            }
        }
    } else {
        Vector3d n[2];
        int t = 0;
        for (list<ContactGraphNode*>::iterator it = node->bedges.begin();
             it != node->bedges.end(); ++it, ++it1) {
            if (level[(*it)->inComponent] != -1 && level[(*it)->inComponent] <= level[node->inComponent] && (*it1)->collide_handling(0, false)) {
                if (t == 2) {
                    node->rgd->v = Vector3d(0, 0, 0);
                } else {
                    n[t++] = (*it1)->n;
                }
            }
        }
        n[0] = n[0].crossProduct(n[1]);
        node->rgd->v = n[0] * node->rgd->v.dotProduct(n[0]);
    }
    return res;
}

void ContactGraph::tarjan(ContactGraph::ContactGraphNode *u) {
    u->instack = 2;
    u->low = u->dfn = index++;
    stack.push_back(u);
    for (list<ContactGraphNode*>::iterator it = u->edges.begin(); it != u->edges.end(); ++it) {
        if ((*it)->dfn == -1) {
            tarjan((*it));
            u->low = min(u->low, (*it)->low);
        } else
            if ((*it)->instack == 2) {
                u->low = min(u->low, (*it)->dfn);
            }
    }
    if (u->low == u->dfn) {
        components.push_back(vector<ContactGraphNode*>());
        while (stack.size()) {
            ContactGraphNode* cgn = stack.back();
            stack.pop_back();
            cgn->instack = 1;
            components.back().push_back(cgn);
            cgn->inComponent = (int)components.size() - 1;
            if (cgn == u)
                break;
        }
    }
}

void ContactGraph::BuildComponents() {
    index = 0;
    stack.clear();
    for (map<Rigid_Geometry*, ContactGraphNode>::iterator it = graph.begin();
         it != graph.end(); ++it) {
        if (it->second.dfn == -1) {
            tarjan(&(it->second));
        }
    }
    level.resize(components.size(), -1);
    degree.resize(components.size(), 0);
}
