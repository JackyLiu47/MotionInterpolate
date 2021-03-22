#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <float.h>
#include "motion.h"
#include "interpolator.h"
#include "types.h"
#include "transform.h"
#include "iostream"
#include "ctime"

Interpolator::Interpolator()
{
  //Set default interpolation type
  m_InterpolationType = LINEAR;

  //set default angle representation to use for interpolation
  m_AngleRepresentation = EULER;
}

Interpolator::~Interpolator()
{
}

//Create interpolated motion
void Interpolator::Interpolate(Motion * pInputMotion, Motion ** pOutputMotion, int N) 
{
  //Allocate new motion
  clock_t startTime, endTime;
  startTime = clock();
  *pOutputMotion = new Motion(pInputMotion->GetNumFrames(), pInputMotion->GetSkeleton()); 

  //Perform the interpolation
  if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == EULER))
    LinearInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == LINEAR) && (m_AngleRepresentation == QUATERNION))
    LinearInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == EULER))
    BezierInterpolationEuler(pInputMotion, *pOutputMotion, N);
  else if ((m_InterpolationType == BEZIER) && (m_AngleRepresentation == QUATERNION))
    BezierInterpolationQuaternion(pInputMotion, *pOutputMotion, N);
  else
  {
    printf("Error: unknown interpolation / angle representation type.\n");
    exit(1);
  }
  endTime = clock();
  printf("Interpolation time is : %f",((double)(endTime-startTime)/CLOCKS_PER_SEC));
  printf("s\n");
}

void Interpolator::LinearInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);

    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);

    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
        interpolatedPosture.bone_rotation[bone] = startPosture->bone_rotation[bone] * (1-t) + endPosture->bone_rotation[bone] * t;

      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }

    startKeyframe = endKeyframe;
  }

  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Rotation2Euler(double R[9], double angles[3])
{
  double cy = sqrt(R[0]*R[0] + R[3]*R[3]);

  if (cy > 16*DBL_EPSILON) 
  {
    angles[0] = atan2(R[7], R[8]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = atan2(R[3], R[0]);
  } 
  else 
  {
    angles[0] = atan2(-R[5], R[4]);
    angles[1] = atan2(-R[6], cy);
    angles[2] = 0;
  }

  for(int i=0; i<3; i++)
    angles[i] *= 180 / M_PI;
}

void Interpolator::Euler2Rotation(double angles[3], double R[9])
{
  // students should implement this
  double Rx[4][4], Ry[4][4], Rz[4][4],mult[4][4], res[4][4];//rotation matrices and result matrix
  rotationX(Rx,angles[0]);
  rotationY(Ry,angles[1]);
  rotationZ(Rz,angles[2]);
  matrix_mult(Rz,Ry,mult);
  matrix_mult(mult,Rx,res);
  //transfer data from 4*4 matrix to array R[9]
  for(int i=0; i<9; i++){
    int m = 0, n = 0;
    m = i / 3;
    n = i % 3;
    R[i] = res[m][n];
  }
}

void Interpolator::BezierInterpolationEuler(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
  int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1

  int startKeyframe = 0;
  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;
    int prevKeyframe = startKeyframe - N - 1;
    int nextKeyframe = startKeyframe + 2*N + 2;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
    Posture * prevPosture;
    if (prevKeyframe >= 0)
      prevPosture = pInputMotion->GetPosture(prevKeyframe);
    Posture * nextPosture;
    if (nextKeyframe < inputLength)
      nextPosture = pInputMotion->GetPosture(nextKeyframe);
    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);
    // interpolate in between
    
    vector p0, p1, p2, p3;
    vector a,b;//control points
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      p1 = startPosture->root_pos;
      p2 = endPosture->root_pos;

      vector temp;
      //consider different position of frame(start,end and intermediate)
      //calculate an = pn + k(an-pn) and simplify the calculation
      if (startKeyframe == 0)
      {
        p3 = nextPosture->root_pos;
        a = p1 + (p2 + p2 - p3 - p1) * (1.0/3);
        temp = p2 + (p3 - p1) * 0.5 * (1.0/3);
        b = (p2 - temp) + p2;
      }
      else if (nextKeyframe > inputLength)
      {
        p0 = prevPosture->root_pos;
        a = (p1 + p1 - p0 + p2) * 0.5;
        a = p1 + (a - p1) * (1.0/3);
        b = (p1 - p0) + p1;
        b = p2 + (b - p2) * (1.0/3);
      }
      else
      {
        p0 = prevPosture->root_pos;
        p3 = nextPosture->root_pos;
        a = p1 + (p2 - p0) * 0.5 * (1.0/3);
        temp = p2 + (p3 - p1) * 0.5 * (1.0/3);
        b = p2 + (p2 - temp);
      }

      interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a, b, p2);

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        p1 = startPosture->bone_rotation[bone];
        p2 = endPosture->bone_rotation[bone];

      //consider different position of frame(start,end and intermediate)
      //calculate an = pn + k(an-pn) and simplify the calculation
        if (startKeyframe == 0)
        {
          p3 = nextPosture->bone_rotation[bone];
          a = (p2 - p3) + p2;
          a = p1 + (a - p1) * (1.0/3);
          temp = p2 + (p3 - p1) * 0.5 * (1.0/3);
          b = p2 + (p2 - temp);
        }
        else if (nextKeyframe>inputLength)
        {
          p0 = prevPosture->bone_rotation[bone];
          a = p1 + (p2 - p0) * 0.5 * (1.0/3);
          b = (p1 - p0) + p1;
          b = p2 + (b - p2) * (1.0/3);
        }
        else
        {
          p0 = prevPosture->bone_rotation[bone];
          p3 = nextPosture->bone_rotation[bone];
          a = p1 + (p2 - p0) * 0.5 * (1.0/3);
          temp = p2 + (p3 - p1) * 0.5 * (1.0/3);
          b = p2 + (p2 - temp);
        }
        interpolatedPosture.bone_rotation[bone] = DeCasteljauEuler(t, p1, a, b, p2);
      }
      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }
    startKeyframe = endKeyframe;
  }
  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::LinearInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
    int inputLength = pInputMotion->GetNumFrames(); // frames are indexed 0, ..., inputLength-1
    
    int startKeyframe = 0;
    Quaternion<double> qStart, qEnd, qInter;
    while (startKeyframe + N + 1 < inputLength)
    {
        int endKeyframe = startKeyframe + N + 1;
        
        Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
        Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
        
        // copy start and end keyframe
        pOutputMotion->SetPosture(startKeyframe, *startPosture);
        pOutputMotion->SetPosture(endKeyframe, *endPosture);
        
        // interpolate in between
        for(int frame=1; frame<=N; frame++)
        {
            Posture interpolatedPosture;
            double t = 1.0 * frame / (N+1);
            
            // interpolate root position
            interpolatedPosture.root_pos = startPosture->root_pos * (1-t) + endPosture->root_pos * t;
            
            // interpolate bone rotations
            for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++){
                Euler2Quaternion(startPosture->bone_rotation[bone].p, qStart);
                Euler2Quaternion(endPosture->bone_rotation[bone].p, qEnd);
                qInter = Slerp(t, qStart, qEnd);
                Quaternion2Euler(qInter, interpolatedPosture.bone_rotation[bone].p);
            }
            pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
        }
        startKeyframe = endKeyframe;
    }
    
    for(int frame=startKeyframe+1; frame<inputLength; frame++)
        pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::BezierInterpolationQuaternion(Motion * pInputMotion, Motion * pOutputMotion, int N)
{
  // students should implement this
  int inputLength = pInputMotion->GetNumFrames();

  int startKeyframe = 0;
  vector p0, p1, p2, p3;
  vector a,b;
  Quaternion<double> q0, q1, q2, q3;
  Quaternion<double> qa, qb;

  while (startKeyframe + N + 1 < inputLength)
  {
    int endKeyframe = startKeyframe + N + 1;
    int prevKeyframe = startKeyframe - N - 1;
    int nextKeyframe = startKeyframe + 2*N + 2;

    Posture * startPosture = pInputMotion->GetPosture(startKeyframe);
    Posture * endPosture = pInputMotion->GetPosture(endKeyframe);
    Posture * prevPosture;
    if (prevKeyframe >= 0)
      prevPosture = pInputMotion->GetPosture(prevKeyframe);
    Posture * nextPosture;
    if (nextKeyframe < inputLength)
      nextPosture = pInputMotion->GetPosture(nextKeyframe);
    // copy start and end keyframe
    pOutputMotion->SetPosture(startKeyframe, *startPosture);
    pOutputMotion->SetPosture(endKeyframe, *endPosture);
    
    // interpolate in between
    for(int frame=1; frame<=N; frame++)
    {
      Posture interpolatedPosture;
      double t = 1.0 * frame / (N+1);

      // interpolate root position
      p1 = startPosture->root_pos;
      p2 = endPosture->root_pos;

      vector temp;
      //consider different position of frame(start,end and intermediate)
      //calculate an = pn + k(an-pn) and simplify the calculation
      if (startKeyframe == 0)
      {
        p3 = nextPosture->root_pos;
        a = p1 + (p2 + p2 - p3 - p1) * (1.0/3);
        b = (p1 - p3) * 0.5 * (1.0/3) + p2;
      }
      else if (nextKeyframe>inputLength)
      {
        p0 = prevPosture->root_pos;
        a = p1 + (p2 - p0) * 0.5 * (1.0/3);
        b = p2 + (p1 + p1 - p0 - p2) * (1.0/3);
      }
      else
      {
        p0 = prevPosture->root_pos;
        p3 = nextPosture->root_pos;
        a = p1 + (p2 - p0) * 0.5 * (1.0/3);
        temp = p2 + (p3 - p1) * 0.5 * (1.0/3);
        b = p2 + (p2 - temp);
      }
      interpolatedPosture.root_pos = DeCasteljauEuler(t, p1, a, b, p2);

      // interpolate bone rotations
      for (int bone = 0; bone < MAX_BONES_IN_ASF_FILE; bone++)
      {
        Euler2Quaternion(startPosture->bone_rotation[bone].p, q1);
        Euler2Quaternion(endPosture->bone_rotation[bone].p, q2);

        Quaternion<double> qtemp;
        Quaternion<double> doub;

      //consider different position of frame(start,end and intermediate)
        if (startKeyframe == 0)
        {
          Euler2Quaternion(nextPosture->bone_rotation[bone].p, q3);
          qa = Double(q3, q2);
          qa = Slerp(1.0/3, q1, qa);
          doub = Double(q1, q2);
          qtemp = Slerp(0.5, doub, q3);
          qb = Slerp(-1.0/3, q2, qtemp);
        }
        else if (nextKeyframe>inputLength)
        {
          Euler2Quaternion(prevPosture->bone_rotation[bone].p, q0);
          doub = Double(q0, q1);
          qa = Slerp(0.5, doub, q2);
          qa = Slerp(1.0/3, q1, qa);
          qb = Double(q0, q1);
          qb = Slerp(1.0/3, q2, qb);
        }
        else
        {
          Euler2Quaternion(prevPosture->bone_rotation[bone].p, q0);
          Euler2Quaternion(nextPosture->bone_rotation[bone].p, q3);
          doub = Double(q0, q1);
          qa = Slerp(0.5, doub, q2);
          qa = Slerp(1.0/3, q1, qa);
          doub = Double(q1, q2);
          qtemp = Slerp(0.5, doub, q3);
          qb = Slerp(-1.0/3, q2, qtemp);
        }
        doub = DeCasteljauQuaternion(t, q1, qa, qb, q2);
        Quaternion2Euler(doub, interpolatedPosture.bone_rotation[bone].p);
      }
      pOutputMotion->SetPosture(startKeyframe + frame, interpolatedPosture);
    }
    startKeyframe = endKeyframe;
  }
  for(int frame=startKeyframe+1; frame<inputLength; frame++)
    pOutputMotion->SetPosture(frame, *(pInputMotion->GetPosture(frame)));
}

void Interpolator::Euler2Quaternion(double angles[3], Quaternion<double> & q) 
{
  // students should implement this
  double R[9];
  Euler2Rotation(angles, R);
  q = Quaternion<double>::Matrix2Quaternion(R);
  //euler to rotation matrix then convert matrix to quaternion
  q.Normalize();
}

void Interpolator::Quaternion2Euler(Quaternion<double> & q, double angles[3]) 
{
  // students should implement this
  double R[9];
  q.Quaternion2Matrix(R);
  Rotation2Euler(R, angles);
  //quaternion to rotation matrix then convert to Euler angel
}

Quaternion<double> Interpolator::Slerp(double t, Quaternion<double> & qStart, Quaternion<double> & qEnd_)
{
  // students should implement this
  Quaternion<double> result;
    double cos_theta = qStart.Getx()*qEnd_.Getx() + qStart.Gety()*qEnd_.Gety() + qStart.Getz()*qEnd_.Getz() + qStart.Gets()*qEnd_.Gets();
    double theta = acos(cos_theta);
    //judge if the dot product is negative
    if (cos_theta < 0) {
        theta = M_PI - acos(-cos_theta);
        qEnd_ = -1.0 * qEnd_;
    }
    if (sin(theta) == 0.0) {
        result = qStart;
        return result;
    }
    
    result = sin((1-t)*theta)/sin(theta) * qStart + sin(t*theta)/sin(theta) * qEnd_;
    result.Normalize();
  return result;
}

Quaternion<double> Interpolator::Double(Quaternion<double> p, Quaternion<double> q)
{
  // students should implement this
  Quaternion<double> result;
  double cos_theta;
  cos_theta = p.Getx() * q.Getx() + p.Gety() * q.Gety() + p.Getz() * q.Getz() + p.Gets() * q.Gets();
  result = 2 * cos_theta * q - p;
  return result;
}

vector Interpolator::DeCasteljauEuler(double t, vector p0, vector p1, vector p2, vector p3)
{
  // students should implement this
  vector result;
  vector temp1, temp2, res1, res2;
    temp1 = p0 + (p1 - p0) * t;
    temp2 = p1 + (p2 - p1) * t;
    res1 = temp1 + (temp2 - temp1) * t;
    temp1 = p2 + (p3 - p2) * t;
    res2 = temp2 + (temp1 - temp2) * t;
    result = res1 + (res2 - res1) * t;
  return result;
}

Quaternion<double> Interpolator::DeCasteljauQuaternion(double t, Quaternion<double> p0, Quaternion<double> p1, Quaternion<double> p2, Quaternion<double> p3)
{
  // students should implement this
  Quaternion<double> result;
  Quaternion<double> temp1, temp2, res1, res2;
    temp1 = Slerp(t, p0, p1);
    temp2 = Slerp(t, p1, p2);
    res1 = Slerp(t, temp1, temp2);
    temp1 = Slerp(t, p2, p3);
    res2 = Slerp(t, temp2, temp1);
    result = Slerp(t,res1, res2);
  return result;
}

