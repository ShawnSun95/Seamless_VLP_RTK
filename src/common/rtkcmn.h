/*
 * Derived from RTKLIB 2.4.3 Betas
 */

extern double epoch2time(const double *ep)
{
    const int doy[]={1,32,60,91,121,152,182,213,244,274,305,335};
    double time=0;
    int days,year=(int)ep[0],mon=(int)ep[1],day=(int)ep[2];
    double sec=ep[5];
    
    if (year<1970||2099<year||mon<1||12<mon) return time;
    
    /* leap year if year%4==0 in 1901-2099 */
    days=(year-1970)*365+(year-1969)/4+doy[mon-1]+day-2+(year%4==0&&mon>=3?1:0);
    time=days*86400+(int)ep[3]*3600+(int)ep[4]*60+sec;
    return time;
}