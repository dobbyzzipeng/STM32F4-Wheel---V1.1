#ifndef _RTK_H_
#define _RTK_H_

void llh2ecef(const double *pos, double *r);
double DmToDd(unsigned long long data);
double rtk_dis_analysis(double lon1,double lat1,double lon2,double lat2);
#endif
