#include "Contact.h"
#include "vmath.h"
#include "Rigid_Geometry.h"
#include "Solver.h"
using namespace SimLib;

void Contact::contact_handling(std::vector<Contact>& contacts) {
    ARRAY<1,double> b((int)contacts.size());
    for (int i = 0; i < contacts.size(); ++i) {
        Contact& c = contacts[i];
        c.support = 0;
        Rigid_Geometry *A = c.a, *B = c.b;
        Vector3d &n = c.n, &ra = c.ra, &rb = c.rb;
        Vector3d &f_a = A->f, &f_b = B->f, &t_a = A->M, &t_b = B->M;
        double m1 = A->nailed ? 0 : 1 / A->mass;
        double m2 = B->nailed ? 0 : 1 / B->mass;
        Matrix3d r1 = A->rotation.rotMatrix();
        Matrix3d r2 = B->rotation.rotMatrix();
        Matrix3d J1 = A->nailed ? Matrix3d::createScale(0, 0, 0) : r1 * A->J * r1.transpose();
        Matrix3d J2 = B->nailed ? Matrix3d::createScale(0, 0, 0) : r2 * B->J * r2.transpose();
        Vector3d a_ext_part = f_a * m1 + ((J1 * t_a).crossProduct(ra));
        Vector3d b_ext_part = f_b * m2 + ((J2 * t_b).crossProduct(rb));
        Vector3d a_vel_part = A->w.crossProduct(A->w.crossProduct(ra)) + (J1 * (r1 * A->J0 * r1.transpose() * A->w)).crossProduct(ra);
        Vector3d b_vel_part = B->w.crossProduct(B->w.crossProduct(rb)) + (J2 * (r2 * B->J0 * r2.transpose() * B->w)).crossProduct(rb);
        double k1 = n.dotProduct(((a_ext_part + a_vel_part) - (b_ext_part + b_vel_part)));
        double k2 = 2 * B->w.crossProduct(n).dotProduct(A->v + A->w.crossProduct(ra) - B->v - B->w.crossProduct(rb));
        b(i+1) = k1 + k2;
    }
    ARRAY<2, double> a((int)contacts.size(), (int)contacts.size());
    for (int i = 0; i < contacts.size(); ++i) {
        for (int j = 0; j < contacts.size(); ++j) {
            Contact &ci = contacts[i];
            Contact &cj = contacts[j];
            if (ci.a != cj.a && ci.b != cj.b && ci.a != cj.b && ci.b != cj.a) {
                a(i+1,j+1) = 0;
                continue;
            }
            Rigid_Geometry *A = ci.a, *B = ci.b;
            Vector3d &ni = ci.n, &nj = cj.n, &pi = ci.p, &pj = cj.p, &ra = ci.ra, &rb = ci.rb;
            ci.ra = pi - ci.a->x;
            ci.rb = pi - ci.b->x;
            cj.ra = pj - cj.a->x;
            cj.rb = pj - cj.b->x;
            Vector3d force_a, force_b, torque_a, torque_b;
            if (cj.a == ci.a) {
                force_a = nj;
                torque_a = cj.ra.crossProduct(nj);
            } else {
                if (cj.b == ci.a) {
                    force_a = -nj;
                    torque_a = cj.rb.crossProduct(-nj);
                }
            }
            if (cj.a == ci.b) {
                force_b = nj;
                torque_b = cj.ra.crossProduct(nj);
            } else {
                if (cj.b == ci.b) {
                    force_b = -nj;
                    torque_b = cj.rb.crossProduct(-nj);
                }
            }
            
            double m1 = A->nailed ? 0 : 1 / A->mass;
            double m2 = B->nailed ? 0 : 1 / B->mass;
            Matrix3d r1 = A->rotation.rotMatrix();
            Matrix3d r2 = B->rotation.rotMatrix();
            Matrix3d J1 = A->nailed ? Matrix3d::createScale(0, 0, 0) : r1 * A->J * r1.transpose();
            Matrix3d J2 = B->nailed ? Matrix3d::createScale(0, 0, 0) : r2 * B->J * r2.transpose();
            Vector3d a_linear = force_a * m1, a_angular = (J1 * torque_a).crossProduct(ra);
            Vector3d b_linear = force_b * m2, b_angular = (J2 * torque_b).crossProduct(rb);
            a(i+1,j+1) = ni.dotProduct((a_linear + a_angular) - (b_linear + b_angular));
        }
    }
    ARRAY<1, double> c = b;
    for (int i = 0; i < 50; ++i) {
        bool flag = false;
        for (int j = 1; j <= contacts.size(); ++j) {
            double f_n = c(j) == 0 ? 0 : -c(j) / a(j,j);
            if (f_n + contacts[j-1].support < 0)
                f_n = -contacts[j-1].support;
            if (abs(f_n) > 1e-3)
                flag = true;
            for (int k = 1; k <= contacts.size(); ++k)
                c(k) += a(j,k) * f_n;
            contacts[j-1].support += f_n;
        }
        if (!flag)
            break;
    }
}