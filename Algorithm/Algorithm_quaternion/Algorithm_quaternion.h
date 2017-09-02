#ifndef __Algorithm_quaternion_H
#define	__Algorithm_quaternion_H

typedef   struct {
  float Pitch;
  float Roll;
  float Yaw;
} EulerAngle;

typedef volatile struct {
  float q0;
  float q1;
  float q2;
  float q3;
} Quaternion;

typedef volatile struct {
  float x;
  float y;
  float z;
} Gravity;

Gravity Quaternion_vectorGravity( Quaternion *pNumQ );
void Quaternion_ToNumQ( Quaternion *pNumQ, EulerAngle *pAngE );
void Quaternion_ToAngE( Quaternion *pNumQ, EulerAngle *pAngE, float *_matrix);
Quaternion Quaternion_Multiply( Quaternion NowQ, Quaternion OldQ );
void Quaternion_Normalize( Quaternion *pNumQ );
void Quaternion_RungeKutta( Quaternion *pNumQ, float GyrX, float GyrY, float GyrZ, float helfTimes );
void Quaternion_ToMatrix(Quaternion *pNumQ, float *_matrix);
#endif /* __Algorithm_quaternion_H */
