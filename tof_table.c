#include <stdint.h>
#include "tof_table.h"

const int16_t tof_tbl[TOF_TBL_LEN] =
{
/*i=   0*/     0,
/*i=   1*/    14,
/*i=   2*/    28,
/*i=   3*/    42,
/*i=   4*/    56,
/*i=   5*/    70,
/*i=   6*/    84,
/*i=   7*/    98,
/*i=   8*/   112,
/*i=   9*/   126,
/*i=  10*/   140,
/*i=  11*/   154,
/*i=  12*/   168,
/*i=  13*/   182,
/*i=  14*/   196,
/*i=  15*/   209,
/*i=  16*/   223,
/*i=  17*/   237,
/*i=  18*/   251,
/*i=  19*/   265,
/*i=  20*/   279,
/*i=  21*/   293,
/*i=  22*/   307,
/*i=  23*/   321,
/*i=  24*/   335,
/*i=  25*/   348,
/*i=  26*/   362,
/*i=  27*/   376,
/*i=  28*/   390,
/*i=  29*/   404,
/*i=  30*/   417,
/*i=  31*/   431,
/*i=  32*/   445,
/*i=  33*/   459,
/*i=  34*/   473,
/*i=  35*/   486,
/*i=  36*/   500,
/*i=  37*/   514,
/*i=  38*/   527,
/*i=  39*/   541,
/*i=  40*/   555,
/*i=  41*/   568,
/*i=  42*/   582,
/*i=  43*/   596,
/*i=  44*/   609,
/*i=  45*/   623,
/*i=  46*/   636,
/*i=  47*/   650,
/*i=  48*/   663,
/*i=  49*/   677,
/*i=  50*/   690,
/*i=  51*/   704,
/*i=  52*/   717,
/*i=  53*/   731,
/*i=  54*/   744,
/*i=  55*/   757,
/*i=  56*/   771,
/*i=  57*/   784,
/*i=  58*/   797,
/*i=  59*/   811,
/*i=  60*/   824,
/*i=  61*/   837,
/*i=  62*/   850,
/*i=  63*/   863,
/*i=  64*/   877,
/*i=  65*/   890,
/*i=  66*/   903,
/*i=  67*/   916,
/*i=  68*/   929,
/*i=  69*/   942,
/*i=  70*/   955,
/*i=  71*/   968,
/*i=  72*/   981,
/*i=  73*/   994,
/*i=  74*/  1007,
/*i=  75*/  1020,
/*i=  76*/  1033,
/*i=  77*/  1046,
/*i=  78*/  1058,
/*i=  79*/  1071,
/*i=  80*/  1084,
/*i=  81*/  1097,
/*i=  82*/  1109,
/*i=  83*/  1122,
/*i=  84*/  1135,
/*i=  85*/  1147,
/*i=  86*/  1160,
/*i=  87*/  1172,
/*i=  88*/  1185,
/*i=  89*/  1197,
/*i=  90*/  1210,
/*i=  91*/  1222,
/*i=  92*/  1235,
/*i=  93*/  1247,
/*i=  94*/  1259,
/*i=  95*/  1272,
/*i=  96*/  1284,
/*i=  97*/  1296,
/*i=  98*/  1308,
/*i=  99*/  1320,
/*i= 100*/  1333,
/*i= 101*/  1345,
/*i= 102*/  1357,
/*i= 103*/  1369,
/*i= 104*/  1381,
/*i= 105*/  1393,
/*i= 106*/  1405,
/*i= 107*/  1417,
/*i= 108*/  1429,
/*i= 109*/  1440,
/*i= 110*/  1452,
/*i= 111*/  1464,
/*i= 112*/  1476,
/*i= 113*/  1488,
/*i= 114*/  1499,
/*i= 115*/  1511,
/*i= 116*/  1522,
/*i= 117*/  1534,
/*i= 118*/  1546,
/*i= 119*/  1557,
/*i= 120*/  1569,
/*i= 121*/  1580,
/*i= 122*/  1591,
/*i= 123*/  1603,
/*i= 124*/  1614,
/*i= 125*/  1625,
/*i= 126*/  1637,
/*i= 127*/  1648,
/*i= 128*/  1659,
/*i= 129*/  1670,
/*i= 130*/  1681,
/*i= 131*/  1693,
/*i= 132*/  1704,
/*i= 133*/  1715,
/*i= 134*/  1726,
/*i= 135*/  1737,
/*i= 136*/  1748,
/*i= 137*/  1758,
/*i= 138*/  1769,
/*i= 139*/  1780,
/*i= 140*/  1791,
/*i= 141*/  1802,
/*i= 142*/  1812,
/*i= 143*/  1823,
/*i= 144*/  1834,
/*i= 145*/  1844,
/*i= 146*/  1855,
/*i= 147*/  1865,
/*i= 148*/  1876,
/*i= 149*/  1886,
/*i= 150*/  1897,
/*i= 151*/  1907,
/*i= 152*/  1917,
/*i= 153*/  1928,
/*i= 154*/  1938,
/*i= 155*/  1948,
/*i= 156*/  1958,
/*i= 157*/  1969,
/*i= 158*/  1979,
/*i= 159*/  1989,
/*i= 160*/  1999,
/*i= 161*/  2009,
/*i= 162*/  2019,
/*i= 163*/  2029,
/*i= 164*/  2039,
/*i= 165*/  2049,
/*i= 166*/  2059,
/*i= 167*/  2068,
/*i= 168*/  2078,
/*i= 169*/  2088,
/*i= 170*/  2098,
/*i= 171*/  2107,
/*i= 172*/  2117,
/*i= 173*/  2127,
/*i= 174*/  2136,
/*i= 175*/  2146,
/*i= 176*/  2155,
/*i= 177*/  2165,
/*i= 178*/  2174,
/*i= 179*/  2184,
/*i= 180*/  2193,
/*i= 181*/  2202,
/*i= 182*/  2212,
/*i= 183*/  2221,
/*i= 184*/  2230,
/*i= 185*/  2239,
/*i= 186*/  2248,
/*i= 187*/  2258,
/*i= 188*/  2267,
/*i= 189*/  2276,
/*i= 190*/  2285,
/*i= 191*/  2294,
/*i= 192*/  2303,
/*i= 193*/  2312,
/*i= 194*/  2321,
/*i= 195*/  2329,
/*i= 196*/  2338,
/*i= 197*/  2347,
/*i= 198*/  2356,
/*i= 199*/  2365,
/*i= 200*/  2373,
/*i= 201*/  2382,
/*i= 202*/  2391,
/*i= 203*/  2399,
/*i= 204*/  2408,
/*i= 205*/  2416,
/*i= 206*/  2425,
/*i= 207*/  2433,
/*i= 208*/  2442,
/*i= 209*/  2450,
/*i= 210*/  2458,
/*i= 211*/  2467,
/*i= 212*/  2475,
/*i= 213*/  2483,
/*i= 214*/  2492,
/*i= 215*/  2500,
/*i= 216*/  2508,
/*i= 217*/  2516,
/*i= 218*/  2524,
/*i= 219*/  2532,
/*i= 220*/  2540,
/*i= 221*/  2548,
/*i= 222*/  2556,
/*i= 223*/  2564,
/*i= 224*/  2572,
/*i= 225*/  2580,
/*i= 226*/  2588,
/*i= 227*/  2596,
/*i= 228*/  2604,
/*i= 229*/  2612,
/*i= 230*/  2619,
/*i= 231*/  2627,
/*i= 232*/  2635,
/*i= 233*/  2642,
/*i= 234*/  2650,
/*i= 235*/  2658,
/*i= 236*/  2665,
/*i= 237*/  2673,
/*i= 238*/  2680,
/*i= 239*/  2688,
/*i= 240*/  2695,
/*i= 241*/  2703,
/*i= 242*/  2710,
/*i= 243*/  2717,
/*i= 244*/  2725,
/*i= 245*/  2732,
/*i= 246*/  2739,
/*i= 247*/  2747,
/*i= 248*/  2754,
/*i= 249*/  2761,
/*i= 250*/  2768,
/*i= 251*/  2775,
/*i= 252*/  2782,
/*i= 253*/  2789,
/*i= 254*/  2797,
/*i= 255*/  2804
};
