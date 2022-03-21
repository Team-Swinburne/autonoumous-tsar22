#pragma once
#include <iostream>
#include <cmath>
#include <vector>

using namespace std;

class Degree_State {
    public:
    const double pi = acos(-1);

    int mx;        // = startingPos.x
    int my;        // = startingPos.y 

    double aDeg;                // = starting_heading
    double a = rad(aDeg);       // = rad(starting_heading) 

    double x1;     // x pos in space of 10x10 cells
    double y1;     // y pos in space of 10x10 cells
    
    
    double dDeg;   // d < 0 = left turn, d > 0 = right turn, d = 0 = no turn    
    double d; 

    double a2;      // angle after turn
    double a2Deg;   // angle after turn (deg)
    double b;       // angle/heading after 100cm if d/2
    double p1;      // angle perpendicular to a
    double p2;      // angle perpendicular to a2

    double xr;      // x pos of circle with turning radius
    double yr;      // y pos of circle with turning radius
    double r;       // radius of circle that forms turn
    int r0 = 500;  // radius of the circle used to measure a2

    double x2;      // x pos after 100cm move
    double y2;      // y pos after 100cm move

    double a3Deg;    // new a (deg)
    int mx2;        // new map pos x
    int my2;        // new map pos y
    
    // Default Constructor - shouldn't be used used, just to initialise a Degree_State object as a property of other classes
    Degree_State();

    // Initial and Between States Constructor
    Degree_State(int startingPosx, int startingPosy, double startHeading) {
        mx = startingPosx;
        my = startingPosy;
        aDeg = startHeading;
        // cout << "DegState:  x = " << mx << ",   y = " << my << ",   startHeading = " << startHeading << endl;

        x1 = 50;
        y1 = 50;

        a = rad(aDeg);
        p1 = a - rad(90);
    }

    // Constructor for any move after first
    Degree_State(int startingPosx, int startingPosy, double startHeading, int spacePosx, int spacePosy) {
        mx = startingPosx;
        my = startingPosy;
        // cout << "mx = " << mx << ", my = " << my << endl;
        
        aDeg = startHeading;
        a = rad(aDeg);
        p1 = a - rad(90);
        //cout << "a = " << aDeg << endl;

        x1 = spacePosx; 
        y1 = spacePosy;

        //cout << "x1 = " << x1 << endl;
        //cout << "y1 = " << y1 << endl;

        // cout << "DegState:  x = " << startingPosx << ",   y = " << startingPosy << ",   startHeading = " << startHeading << 
        //     ",  spacePosx = " << spacePosx << ",  spacePosy = " << spacePosy << endl;       
    }

    // Copy Constructor
    Degree_State(const Degree_State& aDegreeState)
    {
        mx = aDegreeState.mx;
        my = aDegreeState.my;
        x1 = aDegreeState.x1;
        y1 = aDegreeState.y1;
        aDeg = aDegreeState.aDeg;
        a = aDegreeState.a;
        p1 = aDegreeState.p1;
    }

    // Assignment Operator
    Degree_State& operator=(const Degree_State& aDegreeState) {
        if (&aDegreeState != this) {
            mx = aDegreeState.mx;
            my = aDegreeState.my;
            x1 = aDegreeState.x1;
            y1 = aDegreeState.y1;
            aDeg = aDegreeState.aDeg;
            a = aDegreeState.a;  
            p1 = aDegreeState.p1;      
        }
        return *this;
    }

    double rad(double degree) {
        return (degree*pi)/180;
    }
    double deg(double radian) {
        return (radian*180)/pi;
    }

    bool isUndef(double num) { 
        if (num != num) {
            return true;
        }
        else {
            return false;
        }
    }

    bool pointsAreValid(float x2, float y2, float x1, float y1, int ll, int rl, int bl, int ul) {
        if (ll <= x2 && x2 <= rl && bl <= y2 && y2 <= ul && (x2 != x1 || y2 != y1)) {
            //cout << "points are valid" << endl;
            return true;
        }
        else {
            //cout << "points are NOT valid" << endl;
            return false;
        }
    }

    Degree_State* move(double movingDeg) {
        dDeg = movingDeg;             
        d = rad(dDeg); 

        //cout << "MOVING BY " << dDeg << " DEGREES" << endl;

        a2 = a - d;
        a2Deg = round(deg(a2));
        b = (a2 - a)/2;
        p2 = a2 - rad(90);

        // cout << "a2 = " << a2 << endl;
        // cout << "a = " << a << endl;
        // cout << "b = " << b << endl;

        //cout << "tan(p1) = " << tan(p1) << endl;
        //cout << "tan(p2) = " << tan(p2) << endl;

        if (tan(p1) > 5000 || tan(p1) < -5000) {                   // if tan(p1) undefined but for some reason i'm not getting undefined on here
            //cout << "tan(p1) undefined" << endl;
            xr = x1;
            yr = tan(p2)*(-r0*cos(a+b))+r0*sin(a+b) + y1;
        }
        else if (tan(p2) > 5000 || tan(p2) < -5000) {              // if tan(p2) undefined, same deal as above
            //cout << "tan(p2) undefined" << endl;
            xr = r0*cos(a+b) + x1;
            yr = tan(p1)*(r0*cos(a+b)) + y1;
        }
        else {                                  // otherwise if neither of those are undefined
            //cout << "all g" << endl;
            xr = (tan(p2)*r0*cos(a+b) + tan(p2)*x1 - r0*sin(a+b) - tan(p1)*x1) / (tan(p2) - tan(p1));
            yr = tan(p1)*(xr-x1) + y1;
        }
        if (d == 0){
            //cout << "no circle, going straight" << endl;
        }
        else {
            //cout << "xr = " << xr << endl;
            //cout << "yr = " << yr << endl;
            r = hypot((xr-x1), (yr-y1));
            //cout << "r = " << r << endl;
        }

        x2 = r0 * cos(a+b) + x1;
        y2 = r0 * sin(a+b) + y1;

        // cout << "x2 = " << x2 << endl;
        // cout << "y2 = " << y2 << endl;
        a3Deg = fmod(a2Deg+360, 360);
        // cout << "a3 = " << a3Deg << endl;
        
        double x2r = round(x2);
        double y2r = round(y2);

        int newx1 = fmod(x2r+1000, 100);
        int newy1 = fmod(y2r+1000, 100);

        // cout << "x2 rounded = " << x2r << endl;
        // cout << "y2 rounded = " << y2r << endl;
        
        mx2 = floor(mx + (x2/100));
        my2 = floor(my - floor(y2/100) + (newy1/100));

        // cout << "mx2 = " << mx2 << endl;
        // cout << "my2 = " << my2 << endl;
                
        Degree_State* newDegState = new Degree_State(mx2, my2, a3Deg, newx1, newy1);
        return newDegState;
    }

    // to get path states for each cell from start to finish 
    vector<Degree_State*> GetBetweenStates(double movingDeg) {      
        move(movingDeg);
        vector<Degree_State*> Between_States;
        int bmx = mx, bmy = my;
        double scx = x1, scy = y1, fcx, fcy, ba2;
        int bl = 0, ul = 100; 
        int ll = 0, rl = 100;

        int loop = 0, fcount = 0;
        bool pchanged = false;
        while (true) {
            vector<double> fcxoptions, fcyoptions;
            double e35 = ((ul-y1)/tan(a+b)) + x1;
            fcxoptions.push_back(e35);
            fcyoptions.push_back(ul);
            double e75 = tan(a+b)*(rl-x1) + y1;
            fcxoptions.push_back(rl);
            fcyoptions.push_back(e75);
            double e15 = ((bl-y1)/tan(a+b)) + x1;
            fcxoptions.push_back(e15);
            fcyoptions.push_back(bl);
            double e55 = tan(a+b)*(ll-x1) + y1;
            fcxoptions.push_back(ll);
            fcyoptions.push_back(e55);

         
            // cout << "       ul = " << ul << endl;
            // cout << "ll = " << ll << "      rl = " << rl << endl;
            // cout << "       bl = " << bl << endl;

            int tll = ll, trl = rl, tbl = bl, tul = ul;
            int vcount = 0;
            for (int i = 0; i < 4; i++) {
                // cout << i << endl;
                if (pointsAreValid(fcxoptions[i], fcyoptions[i], scx, scy, ll, rl, bl, ul)) {
                    // cout << i << " is valid" << endl;
                    fcx = fcxoptions[i];
                    fcy = fcyoptions[i];
                    //cout << "fcx = " << fcx << ", fcy = " << fcy << endl;
                    // cout << "scx = " << scx << ", scy = " << scy << endl; 
                    // cout << "fcx = " << fcx << ", fcy = " << fcy << endl; 
                    if (i == 0 && my > my2) {   // valid point above and end is higher than start
                        vcount ++;
                        bmy -= 1;
                        tbl += 100;
                        tul += 100;
                    }
                    if (i == 1 && mx < mx2) {   // valid point to the right and end is right from start
                        vcount ++;
                        bmx += 1;
                        tll += 100;
                        trl += 100;
                    }
                    if (i == 2 && my2 > my) {   // valid point below and end is lower than start
                        vcount ++;
                        bmy += 1;
                        tbl -= 100;
                        tul -= 100;
                    }
                    if (i == 3 && mx2 < mx) {   // valid point to the left and end is left from start
                        vcount ++;
                        bmx -= 1;
                        tll -= 100;
                        trl -= 100;
                    }
                }
            }
            if (vcount > 0) {
                fcount = 0;
                ll = tll;
                rl = trl;
                bl = tbl;
                ul = tul;
                if (bmx == mx2 && bmy == my2) {
                // if (round(fcx) == round(x2) && round(fcy) == round(y2)) {
                    break;
                }

                scx = fcx;
                scy = fcy;

                if (d == 0) {
                    ba2 = a;
                }
                else {
                    if ((d < 0 && fcx-xr >= 0) || (d > 0 && fcx-xr < 0)) {
                        ba2 = atan((fcy-yr)/(fcx-xr)) + rad(90);
                    }
                    else {
                        ba2 = atan((fcy-yr)/(fcx-xr)) + rad(270);
                    }
                }

                ba2 = fmod(round(deg(ba2)), 360);
                Degree_State* betweenState = new Degree_State(bmx, bmy, ba2);
                Between_States.push_back(betweenState);
            }
            else {
                fcount++;
                if (fcount == 1) {
                    bl += 100;
                    ul += 100;
                    bmy -= 1;
                    pchanged = true;
                    // cout << "moved box up" << endl;
                }
                else if (fcount == 2) {
                    if (pchanged == true) {
                        bl -= 100;
                        ul -= 100;
                        bmy += 1;
                        Between_States.pop_back();
                        pchanged = false;
                    }
                    ll += 100;
                    rl += 100;
                    bmx += 1;
                    pchanged = true;
                    // cout << "moved box right" << endl;
                }
                else if (fcount == 3) {
                    if (pchanged == true) {
                        ll -= 100;
                        rl -= 100;
                        bmx -= 1;
                        Between_States.pop_back();
                        pchanged = false;
                    }
                    bl -= 100;
                    ul -= 100;
                    bmy += 1;
                    pchanged = true;
                    // cout << "moved box down" << endl;
                }
                else if (fcount == 4) {
                    if (pchanged == true) {
                        bl += 100;
                        ul += 100;
                        bmy -= 1;
                        Between_States.pop_back();
                        pchanged = false;
                    }
                    ll -= 100;
                    rl -= 100;
                    bmx -= 1;
                    pchanged = true;
                    // cout << "moved box left" << endl;
                }
                else {
                    cout << "NO VALID MOVES" << endl;
                    break;
                }
                Between_States.push_back(new Degree_State(bmx, bmy, aDeg));
            }
            
            
        
            loop++;
            if (loop > 30) {
                cout << "BETWEEN STATES ERROR" << endl;

                exit(EXIT_FAILURE);
            }

        }

        // for (int i = 0; i < Between_States.size(); i++) {
        //     cout << "(" << Between_States.at(i)->mx << ", " << Between_States.at(i)->my << "), heading: " << Between_States.at(i)->aDeg << endl;
        // }

        return Between_States;
    }

    vector<Degree_State*> CornerPoints() {
        vector<float> cxdist;
        vector<float> cydist;
        vector<Degree_State*> cornerPoints;
        // accurate values to the car are sLimit = 7, fLimit = 11, bLimit = 18, but it won't work with the current map/path
        int sLimit = 2, fLimit = 3, bLimit = 5;
        float xside = sLimit*cos(p1);   // x value difference after 70cm to the side
        float yside = sLimit*sin(p1);   // y value difference after 70cm to the side
        float xfront = fLimit*cos(a);   // x value difference after 110cm in front
        float yfront = fLimit*sin(a);   // y value difference after 110cm in front
        float xback = -bLimit*cos(a);   // x value difference after 180cm behind
        float yback = -bLimit*sin(a);   // y value difference after 180cm behind

        cxdist.push_back(xfront - xside);   // x value difference at front left corner
        cydist.push_back(yfront - yside);   // y value difference at front left corner
        cxdist.push_back(xfront + xside);   // x value difference at front right corner
        cydist.push_back(yfront + yside);   // y value difference at front right corner
        cxdist.push_back(xback + xside);    // x value difference at back right corner
        cydist.push_back(yback + yside);    // y value difference at back right corner
        cxdist.push_back(xback - xside);    // x value difference at back left corner
        cydist.push_back(yback - yside);    // y value difference at back left corner
        
        for (int i = 0; i < 4; i++) {
            int cmx, cmy, caDeg;
            double cx1, cy1;
            cmx = floor(mx + (x1/100) + cxdist[i]);
            cx1 = (mx + (x1/100) + cxdist[i] - cmx)*100;

            cmy = floor(my + (y1/100) + cydist[i] - 2*floor((y1/100) + cydist[i]));
            cy1 = (my + (y1/100) + cydist[i] - 2*floor((y1/100) + cydist[i]) - cmy)*100;
       
            if (i == 0) { caDeg = fmod(aDeg+270, 360); }        // if front left corner
            else if (i == 1) { caDeg = fmod(aDeg+180, 360); }   // if front right corner
            else if (i == 2) { caDeg = fmod(aDeg+90, 360); }    // if back right corner
            else if (i == 3) { caDeg = aDeg; }                  // if back left corner
            Degree_State* corner = new Degree_State(cmx, cmy, caDeg, cx1, cy1);
            corner->x1 = cx1;
            corner->y1 = cy1;
            cornerPoints.push_back(corner);
        }  
        return cornerPoints;
    }

    vector<Degree_State*> PerimeterStates() {
        vector<Degree_State*> corners = CornerPoints();
        vector<Degree_State*> perimeter; 
        for (int i = 0; i < 4; i++) {
            perimeter.push_back(corners[i]);
            int nextx;
            int nexty;
            if (i < 3) {
                nextx = corners[i+1]->mx;
                nexty = corners[i+1]->my;
            }
            else {
                nextx = corners[0]->mx;
                nexty = corners[0]->my;
            }
            vector<Degree_State*> betweenCorners = corners[i]->BetweenCorners(nextx, nexty);
            for (int j = 0; j < betweenCorners.size(); j++) {
                perimeter.push_back(betweenCorners[j]);
            }
        }
        return perimeter;
    }

    vector<Degree_State*> BetweenCorners(int endx, int endy) {
        vector<Degree_State*> betweenCorners;
        int bmx = mx, bmy = my;
        double scx = x1, scy = y1, fcx, fcy;
        int bl = 0, ul = 100; 
        int ll = 0, rl = 100;

        int loop = 0;
        while (true) {
            vector<float> fcxoptions, fcyoptions;
            float e35 = round((((ul-y1)/tan(a)) + x1)*100)/100;
            fcxoptions.push_back(e35);
            fcyoptions.push_back(ul);
            float e75 = round((tan(a)*(rl-x1) + y1)*100)/100;
            fcxoptions.push_back(rl);
            fcyoptions.push_back(e75);
            float e15 = round((((bl-y1)/tan(a)) + x1)*100)/100;
            fcxoptions.push_back(e15);
            fcyoptions.push_back(bl);
            float e55 = round((tan(a)*(ll-x1) + y1)*100)/100;
            fcxoptions.push_back(ll);
            fcyoptions.push_back(e55);

            // cout << "       ul = " << ul << endl;
            // cout << "ll = " << ll << "      rl = " << rl << endl;
            // cout << "       bl = " << bl << endl;

            int tll = ll, trl = rl, tbl = bl, tul = ul;
            for (int i = 0; i < 4; i++) {
                // cout << i << ": ";
                // cout << "fcx = " << fcxoptions[i] << ", fcy = " << fcyoptions[i] << endl;
                // if (bmx <= 320) {
                //     cout << "   ll <= x2? ll = " << ll << ", x2 = " << fcxoptions[i] << endl;
                //     cout << "   x2 <= rl? x2 = " << fcxoptions[i] << ", rl = " << rl << endl;
                //     cout << "   bl <= y2? bl = " << bl << ", y2 = " << fcyoptions[i] << endl;
                //     cout << "   y2 <= ul? y2 = " << fcyoptions[i] << ", ul = " << ul << endl;
                //     cout << "   (x2 != x1 || y2 != y1)?" << endl;
                //     cout << "       x2 = " << fcxoptions[i] << ", x1 = " << scx << endl;
                //     cout << "       y2 = " << fcyoptions[i] << ", y1 = " << scy << endl;
                // }
                // ll <= x2 && x2 <= rl && bl <= y2 && y2 <= ul && (x2 != x1 || y2 != y1)
                if (pointsAreValid(fcxoptions[i], fcyoptions[i], scx, scy, ll, rl, bl, ul)) {
                    //cout << i << " is valid" << endl;
                    fcx = fcxoptions[i];
                    fcy = fcyoptions[i];
                    // cout << "fcx = " << fcx << ", fcy = " << fcy << endl;
                    //cout << "scx = " << scx << ", scy = " << scy << endl; 
                    //cout << "fcx = " << fcx << ", fcy = " << fcy << endl; 
                    if (i == 0 && my > endy) {   // valid point above and end is higher than start
                        bmy -= 1;
                        tbl += 100;
                        tul += 100;
                    }
                    if (i == 1 && mx < endx) {   // valid point to the right and end is right from start
                        bmx += 1;
                        tll += 100;
                        trl += 100;
                    }
                    if (i == 2 && endy > my) {   // valid point below and end is lower than start
                        bmy += 1;
                        tbl -= 100;
                        tul -= 100;
                    }
                    if (i == 3 && endx < mx) {   // valid point to the left and end is left from start
                        bmx -= 1;
                        tll -= 100;
                        trl -= 100;
                    }
                }
            }

            ll = tll;
            rl = trl;
            bl = tbl;
            ul = tul;
            // cout << "mx2 = " << endx << ", my2 = " << endy << endl;
            if (bmx == endx && bmy == endy) {
            // if (round(fcx) == round(x2) && round(fcy) == round(y2)) {
                break;
            }

            scx = fcx;
            scy = fcy;

            Degree_State* betweenState = new Degree_State(bmx, bmy, aDeg);
            betweenCorners.push_back(betweenState);
            
            loop++;
            if (loop > 50) {
                cout << "CORNER ERROR!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << endl;
                exit(EXIT_FAILURE);
            }
        }

        // for (int i = 0; i < betweenCorners.size(); i++) {
        //     cout << "(" << betweenCorners.at(i)->mx << ", " << betweenCorners.at(i)->my << "), heading: " << betweenCorners.at(i)->aDeg << endl;
        // }

        return betweenCorners;
    }
};