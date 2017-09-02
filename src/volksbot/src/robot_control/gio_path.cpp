#include "gio_path.h"

void CGioController::InitDefault() {
    this->loop_exit = 0;
    this->AXIS_LENGTH = 0.485;
    this->Vm = 1;           //Max speed of robot.
    this->d_y = 0.001;
    this->d_th = 0.34;
    this->kr_max = 2 / AXIS_LENGTH;
    this->u0 = Vm / 2.0;
    this->a = 1.2;
    this->epsilon = 1e-4;
    this->path = new CCurve();
    this->setPose(0.0, 0.0, 0);
    this->setLocalSystem(0.0);
}

void CGioController::setLocalSystem(double ang) {
    ex[0] = cos(ang);
    ex[1] = sin(ang);
    ey[0] = -sin(ang);
    ey[1] = cos(ang);
}

double CGioController::H_case_1(double y, double theta, double u, double alpha, double *gamma) {
    double h_j = SQR(kr_max) / (SQR(theta) * SQR(1 + 2 * alpha));
    *gamma = 2 * alpha * u * sqrt(h_j);
    return h_j;
};

double CGioController::H_case_2(double y, double theta, double u, double alpha, double *gamma) {
    double h_j = 0.5 * (-SQR(theta) / SQR(y) +
                        sqrt(SQR(SQR(theta) / SQR(y)) + 4 * SQR(kr_max) / (SQR(y) * SQR(1 + 2 * alpha))));
    *gamma = 2 * alpha * u * sqrt(h_j);
    return h_j;
};

double CGioController::Compute_W(double y, double theta, double a, double u, int *err) {
    double res, h, gamma;

    if (fabs(y) < epsilon) {
        if (fabs(theta) >= d_th) {
            cerr << "H1_1\n";
            h = H_case_1(y, theta, u, a, &gamma);
            *err = 0;
        } else {
            cerr << "H1_2\n";
            h = H_case_1(y, d_th, u, a, &gamma);
            *err = 0;
        }
    } else {
        if (fabs(y) >= d_y) {
            h = H_case_2(y, d_th, u, a, &gamma);
            *err = 0;
        } else {
            if (fabs(theta) < d_th) {
                h = H_case_2(d_y, d_th, u, a, &gamma);
                *err = 0;
            } else {
                h = H_case_2(d_y, theta, u, a, &gamma);
                *err = 0;
            }
        }
    }

    if (fabs(theta) >= d_th) {
        res = -h * u * y * sin(theta) / theta - gamma * theta;  //reduced steering control law
    } else {
        if (fabs(theta) <= d_th && fabs(theta) >= d_th_0) {
            res = -h * u * y - gamma * theta;
        } else {
            res = -h * u * y;
        }
    }

    return res;
}

CGioController::CGioController() {
    this->InitDefault();
    giofile.open("pos.dat");
};

CGioController::~CGioController() {
    giofile.flush();
    giofile.close();
    giofile.clear();
    delete path;
}

void CGioController::setAxisLength(double val) {
    if (fabs(val) > 0) {
        this->AXIS_LENGTH = fabs(val);
        this->kr_max = 2 / AXIS_LENGTH;
    }
}

double CGioController::getAxisLength() {
    return this->AXIS_LENGTH;
}

void CGioController::setCurrentVelocity(double val, int abs) {
    if (abs) {
        this->u0 = val;
    } else {
        this->u0 = (this->u0 > val) ? (val + this->u0) : (this->u0 + val);
    }
    this->Vm = this->u0 * 2.0;
}

double CGioController::getCurrentVelocity() {
    return this->u0;
}

void CGioController::setPose(double x, double y, double phi) {
    this->x0 = x;
    this->y0 = y;
    this->phi0 = phi;
    NormalizeAngle(this->phi0);
}

void CGioController::getPose(double &x, double &y, double &ph) {
    x = this->x0;
    y = this->y0;
    ph = this->phi0;
}

int CGioController::getPathFromFile(const char *fname) {
    int res = path->LoadFromFile(fname);
    if (res) {
        path->initTraversal();
        this->loop_exit = path->getNext();
    }
    return res;
}

int CGioController::canDetermineRobotPosition(int looped) {
    int exit;
    int check_prev;

    exit = 0;
    check_prev = 0;

    while (this->loop_exit != 0 && !exit) {
        if (path->pointInn(x0, y0)) {
            if (path->getDistanceToEnd(x0, y0) > 0.1) {
                exit = 1;
                giofile << path->getDistanceToEnd(x0, y0) << " "; // u 1
                continue;
            }
        }
        this->loop_exit = path->getNext(looped);
    }

    return exit;
}

int CGioController::getNextState(double &u, double &w, double &vleft, double &vright, int looped) {
    double l, phic = 0, pathAng = 0, tmpw;
    int err;

    if (!canDetermineRobotPosition(looped)) {
        u = 0;
        w = 0;
        vleft = 0;
        vright = 0;
        return 0;
    }

    l = path->getDistance(x0, y0);
    if (path->Evaluate(x0, y0) > 5e-7) {
        l = -l;
    }

    giofile << l << " " << path->Evaluate(x0, y0) << " ";  // u 2 3

    phic = phi0 - path->getAng();
    giofile << phi0 << " " << pathAng << " " << phic << " "; // using 4 5 6
    NormalizeAngle(phic);

    u = this->u0;

    w = Compute_W(l, phic, this->a, u, &err);   //calc angle change of theta;
    std::cerr << "!!!!!!!!!!!!!!!!!!!! W:" << w << std::endl;
    double sign = w < 0 ? -1 : 1;
    w = (fabs(w) > this->Vm / this->AXIS_LENGTH) ? sign * (this->Vm / this->AXIS_LENGTH)
                                                 : w; // Account for robot limitations ???
    //w = (fabs(w) > this->Vm / this->AXIS_LENGTH) ? fabs(this->Vm / this->AXIS_LENGTH) : w;

    giofile << w << " ";
    vright = u + AXIS_LENGTH * w * 0.5;
    vleft = u - AXIS_LENGTH * w * 0.5;

    return 1;
}
