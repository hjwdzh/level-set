#include "Contact.h"
#include "vmath.h"
#include "Rigid_Geometry.h"
#include "Solver.h"
using namespace SimLib;

void Contact::contact_handling(std::vector<Contact>& contacts) {
    ARRAY<1,double> b((int)contacts.size() * (FRICTION_DIM + 2));
    Vector3d r_v(4.3284, 2.3850, 3.2859);
    r_v.normalize();
    for (int i = 0; i < contacts.size(); ++i) {
        Contact& c = contacts[i];
        c.support = 0;
        c.friction.Fill(0);
        c.D(1) = c.n.crossProduct(r_v);
        for (int j = 2; j <= FRICTION_DIM; ++j) {
            c.D(j) = c.D(j-1) + c.n.crossProduct(c.D(j-1)) * tan(3.141592654 * 2 / FRICTION_DIM);
            c.D(j).normalize();
        }
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
        // Force LCP
        b(i * (2 + FRICTION_DIM) + 1) = k1 + k2;
        // Alpha LCP
        for (int j = 1; j <= FRICTION_DIM; ++j) {
            b(i * (2 + FRICTION_DIM) + j + 1) = c.u.dotProduct(c.D(j));
        }
        // Beta LCP
        b((i + 1) * (2 + FRICTION_DIM)) = 0;
    }
    ARRAY<2, double> a((int)contacts.size() * (FRICTION_DIM + 2), (int)contacts.size() * (FRICTION_DIM + 2));
    for (int i = 0; i < contacts.size(); ++i) {
        Contact &ci = contacts[i];
        // Force LCP
        for (int j = 0; j < contacts.size(); ++j) {
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
            ARRAY<1, Vector3d> fric_a(FRICTION_DIM), fric_ta(FRICTION_DIM), fric_b(FRICTION_DIM), fric_tb(FRICTION_DIM);
            if (cj.a == ci.a) {
                force_a = nj;
                torque_a = cj.ra.crossProduct(nj);
                for (int k = 1; k <= FRICTION_DIM; ++k) {
                    fric_a(k) = cj.D(k);
                    fric_ta(k) = cj.ra.crossProduct(cj.D(k));
                }
            } else {
                if (cj.b == ci.a) {
                    force_a = -nj;
                    torque_a = cj.rb.crossProduct(-nj);
                    for (int k = 1; k <= FRICTION_DIM; ++k) {
                        fric_a(k) = -cj.D(k);
                        fric_ta(k) = cj.ra.crossProduct(-cj.D(k));
                    }
                }
            }
            if (cj.a == ci.b) {
                force_b = nj;
                torque_b = cj.ra.crossProduct(nj);
                for (int k = 1; k <= FRICTION_DIM; ++k) {
                    fric_b(k) = cj.D(k);
                    fric_tb(k) = cj.ra.crossProduct(cj.D(k));
                }
            } else {
                if (cj.b == ci.b) {
                    force_b = -nj;
                    torque_b = cj.rb.crossProduct(-nj);
                    for (int k = 1; k <= FRICTION_DIM; ++k) {
                        fric_b(k) = -cj.D(k);
                        fric_tb(k) = cj.ra.crossProduct(-cj.D(k));
                    }
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
            a(i * (2 + FRICTION_DIM) + 1,j * (2 + FRICTION_DIM) + 1) = ni.dotProduct((a_linear + a_angular) - (b_linear + b_angular));
            for (int k = 1; k <= FRICTION_DIM; ++k) {
                Vector3d fa_linear = fric_a(k) * m1, fa_angular = (J1 * fric_ta(k)).crossProduct(ra);
                Vector3d fb_linear = fric_b(k) * m2, fb_angular = (J2 * fric_tb(k)).crossProduct(rb);
                a(i * (2 + FRICTION_DIM) + 1, j * (2 + FRICTION_DIM) + 1 + k) = ni.dotProduct((fa_linear + fa_angular) - (fb_linear + fb_angular));
            }
        }
        //Alpha LCP
        for (int j = 1; j <= FRICTION_DIM; ++j) {
            a(i * (2 + FRICTION_DIM) + 1 + j, (i + 1) * (2 + FRICTION_DIM)) = 1;
        }
        //Beta LCP
        a((i + 1) * (2 + FRICTION_DIM), i * (2 + FRICTION_DIM) + 1) = ci.mu;
        for (int j = 1; j <= FRICTION_DIM; ++j) {
            a((i + 1) * (2 + FRICTION_DIM), i * (2 + FRICTION_DIM) + 1 + j) = -1;
        }
    }
    ARRAY<1, double> c = b;
    ARRAY<1, double> X(c.dim(1));
    for (int i = 0; i < 50; ++i) {
        bool flag = false;
        for (int j = 0; j < contacts.size(); ++j) {
            ARRAY<2, double> A(2 + FRICTION_DIM, 2 + FRICTION_DIM);
            ARRAY<1, double> B(2 + FRICTION_DIM);
            for (int k = 1; k <= 2 + FRICTION_DIM; ++k) {
                B(k) = c(j * (2 + FRICTION_DIM) + k);
                for (int l = 1; l <= 2 + FRICTION_DIM; ++l) {
                    A(k,l) = a(j * (2 + FRICTION_DIM) + k, j * (2 + FRICTION_DIM) + l);
                }
            }
//            memcpy(&B(1), &c(j * (2 + FRICTION_DIM) + 1), sizeof(double) * (2 + FRICTION_DIM));
//            for (int k = 1; k <= 2 + FRICTION_DIM; ++k) {
//                memcpy(&A(k,1), &a(j * (2 + FRICTION_DIM) + k, j * (2 + FRICTION_DIM) + 1), sizeof(double) * (2 + FRICTION_DIM));
//            }
            ARRAY<1, double> f(2 + FRICTION_DIM);
            double minv = 1e30;
            int ind = 0;
            for (int k = 2; k <= FRICTION_DIM + 1; ++k) {
                if (B(k) < minv) {
                    ind = k;
                    minv = B(k);
                }
            }
            //Accurate friction to be done
            f(2 + FRICTION_DIM) = -B(ind);
            f(1) = -B(1) / A(1,1);
            if (f(1) < 0)
                f(1) = 0;
            if (minv < 0) {
                f(ind) = A(2 + FRICTION_DIM,1) * f(1);
            } else {
                
            }
            for (int k = j * (2 + FRICTION_DIM) + 1; k <= (j + 1) * (2 + FRICTION_DIM); ++k) {
                double delta_t = f(k - j * (2 + FRICTION_DIM)) - X(k);
                for (int l = 0; l < contacts.size(); ++l)
                    if (l != j || i == 34) {
                        for (int m = l * (2 + FRICTION_DIM) + 1; m <= (l + 1) * (2 + FRICTION_DIM); ++m) {
                            c(m) += a(m,k) * delta_t;
                        }
                    }
                X(k) += delta_t;
                if (delta_t > 1e-3) {
                    flag = true;
                }
            }
        }
        if (!flag)
            break;
    }

    ARRAY<1, double> ff = b;
    for (int i = 1; i <= b.dim(1); ++i) {
        for (int j = 1; j <= b.dim(1); ++j) {
            ff(i) += a(i,j) * X(j);
        }
    }
    for (int i = 0; i < contacts.size(); ++i) {
        contacts[i].support = X(i * (2 + FRICTION_DIM) + 1);
        for (int j = 1; j <= FRICTION_DIM; ++j) {
            contacts[i].friction(j) = X(i * (2 + FRICTION_DIM) + 1 + j);
            double t = contacts[i].friction(j);
            if (contacts[i].friction(j) != 0) {
                i = i;
            }
        }
    }
}