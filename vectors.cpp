/* 
Copyright 2013 Brad Quick

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "vectors.h"
#include "bradwii.h"

extern globalstruct global;

void vector_cross_product(fixedpointnum *v1, fixedpointnum *v2, fixedpointnum *v3) {
    v3[X_INDEX]=lib_fp_multiply(v1[Y_INDEX],v2[Z_INDEX])-lib_fp_multiply(v1[Z_INDEX],v2[Y_INDEX]);
    v3[Y_INDEX]=lib_fp_multiply(v1[Z_INDEX],v2[X_INDEX])-lib_fp_multiply(v1[X_INDEX],v2[Z_INDEX]);
    v3[Z_INDEX]=lib_fp_multiply(v1[X_INDEX],v2[Y_INDEX])-lib_fp_multiply(v1[Y_INDEX],v2[X_INDEX]);
}

fixedpointnum normalize_vector(fixedpointnum *v) {
    fixedpointnum vectorlengthsquared=lib_fp_multiply(v[0],v[0])+lib_fp_multiply(v[1],v[1])+lib_fp_multiply(v[2],v[2]);
   
    // if we are near zero lenght, choose any unit length vector
    if (vectorlengthsquared<10) {
        v[0]=FIXEDPOINTONE;
        v[1]=v[2]=0;
    } else {
        fixedpointnum multiplier=lib_fp_invsqrt(vectorlengthsquared);

        v[0]=lib_fp_multiply(v[0], multiplier);
        v[1]=lib_fp_multiply(v[1], multiplier);
        v[2]=lib_fp_multiply(v[2], multiplier);
    }
    return(vectorlengthsquared);
}

fixedpointnum vector_dot_product(fixedpointnum *v1, fixedpointnum *v2) {
    return(lib_fp_multiply(v1[0], v2[0])+lib_fp_multiply(v1[1], v2[1])+lib_fp_multiply(v1[2], v2[2]));
}

void rotate_vector_with_small_angles(fixedpointnum *v, fixedpointnum rolldeltaangle, fixedpointnum pitchdeltaangle, fixedpointnum yawdeltaangle) {
   // rotate the attitude by the delta angles.
   // assumes that the delta angles are small angles in degrees and that they are shifted left by TIMESLIVEREXTRASHIFT
   fixedpointnum v_tmp_x=v[X_INDEX];
   fixedpointnum v_tmp_y=v[Y_INDEX];
   fixedpointnum v_tmp_z=v[Z_INDEX];

   // remember that our delta angles are shifted left by TIMESLIVEREXTRASHIFT for resolution.  Take it out here
   v[X_INDEX] += (lib_fp_multiply(rolldeltaangle ,v_tmp_z) - lib_fp_multiply(yawdeltaangle,v_tmp_y))>>(TIMESLIVEREXTRASHIFT);
   v[Y_INDEX] += (lib_fp_multiply(pitchdeltaangle,v_tmp_z) + lib_fp_multiply(yawdeltaangle ,v_tmp_x))>>(TIMESLIVEREXTRASHIFT);
   v[Z_INDEX] -= (lib_fp_multiply(rolldeltaangle,v_tmp_x) + lib_fp_multiply(pitchdeltaangle ,v_tmp_y))>>(TIMESLIVEREXTRASHIFT);
}

// some extra vector functions that aren't currently used.
#ifdef EXTENDEDVECTORFUNCTIONS
void vector_difference_to_euler_angles(fixedpointnum *v1, fixedpointnum *v2, fixedpointnum *euler) {
    // take the difference between the two attitudes and return the euler angles between them
    // find the axis of rotation and angle between the two downVectors
   // the cross products of the two vectors will give us the axis of rotation from one to the other
   fixedpointnum axisofrotation[3];
   vector_cross_product(v1, v2, axisofrotation);

   fixedpointnum axislength=lib_fp_sqrt(normalize_vector(axisofrotation));
   
   // get the angle of rotation between the two vectors
   fixedpointnum angle=lib_fp_atan2(axislength, vector_dot_product(v1, v2));

   fixedpointnum unitvector[3];
   unitvector[0]=0;
   unitvector[1]=FIXEDPOINTONE;
   unitvector[2]=0;
   
   euler[0]=lib_fp_multiply(vector_dot_product(axisofrotation, unitvector), angle);

   unitvector[0]=FIXEDPOINTONE;
   unitvector[1]=0;
   unitvector[2]=0;
   
   euler[1]=lib_fp_multiply(vector_dot_product(axisofrotation, unitvector), angle);
}
   

void rotate_vector_by_axis_angle(fixedpointnum *v1, fixedpointnum *axisvector, fixedpointnum angle, fixedpointnum *v2) {
   fixedpointnum cosineofangle=lib_fp_cosine(angle);
   fixedpointnum sineofangle=lib_fp_sine(angle);
   
   fixedpointnum crossproductvector[3];
   vector_cross_product(axisvector,v1, crossproductvector);
   
   fixedpointnum dotproducttimesoneminuscosineofangle=lib_fp_multiply(vector_dot_product(axisvector,v1),FIXEDPOINTONE-cosineofangle);
   
   v2[0]=lib_fp_multiply(v1[0], cosineofangle)+lib_fp_multiply(crossproductvector[0],sineofangle)+lib_fp_multiply(axisvector[0],dotproducttimesoneminuscosineofangle);
   v2[1]=lib_fp_multiply(v1[1], cosineofangle)+lib_fp_multiply(crossproductvector[1],sineofangle)+lib_fp_multiply(axisvector[1],dotproducttimesoneminuscosineofangle);
   v2[2]=lib_fp_multiply(v1[2], cosineofangle)+lib_fp_multiply(crossproductvector[2],sineofangle)+lib_fp_multiply(axisvector[2],dotproducttimesoneminuscosineofangle);
}
   
void rotate_vector_by_axis_small_angle(fixedpointnum *v1, fixedpointnum *axisvector, fixedpointnum angle) {
     // rotates vector by small angle angle around axis vector.
   fixedpointnum angleinradians=lib_fp_multiply(angle, FIXEDPOINTPIOVER180);
   
   fixedpointnum crossproductvector[3];
   vector_cross_product(axisvector,v1, crossproductvector);
      
   v1[0]+=lib_fp_multiply(crossproductvector[0],angleinradians);
   v1[1]+=lib_fp_multiply(crossproductvector[1],angleinradians);
   v1[2]+=lib_fp_multiply(crossproductvector[2],angleinradians);
}

#endif