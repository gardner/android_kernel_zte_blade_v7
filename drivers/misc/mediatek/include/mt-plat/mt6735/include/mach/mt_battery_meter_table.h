#ifndef _CUST_BATTERY_METER_TABLE_H
#define _CUST_BATTERY_METER_TABLE_H

#define BAT_NTC_10 0
#define BAT_NTC_47 1

#if (BAT_NTC_10 == 1)
#define RBAT_PULL_UP_R    16900
#endif

#if (BAT_NTC_47 == 1)
#define RBAT_PULL_UP_R    61900
#endif

#define RBAT_PULL_UP_VOLT 1800

typedef struct _BATTERY_PROFILE_STRUCT {
	signed int percentage;
	signed int voltage;
} BATTERY_PROFILE_STRUCT, *BATTERY_PROFILE_STRUCT_P;

typedef struct _R_PROFILE_STRUCT {
	signed int resistance;
	signed int voltage;
} R_PROFILE_STRUCT, *R_PROFILE_STRUCT_P;

typedef enum {
	T1_0C,
	T2_25C,
	T3_50C
} PROFILE_TEMPERATURE;

#if (BAT_NTC_10 == 1)
BATT_TEMPERATURE Batt_Temperature_Table[] = {
	{-20, 68237},
	{-15, 53650},
	{-10, 42506},
	{-5, 33892},
	{0, 27219},
	{5, 22021},
	{10, 17926},
	{15, 14674},
	{20, 12081},
	{25, 10000},
	{30, 8315},
	{35, 6948},
	{40, 5834},
	{45, 4917},
	{50, 4161},
	{55, 3535},
	{60, 3014}
};
#endif

#if (BAT_NTC_47 == 1)
	BATT_TEMPERATURE Batt_Temperature_Table[] = {
        {-20,458800},
        {-15,346000},
        {-10,263200},
        { -5,201700},
        {  0,155700},
        {  5,121000},
        { 10,94600},
        { 15,74450},
        { 20,58970},
        { 25,47000},
        { 30,37680},
        { 35,30360},
        { 40,24610},
        { 45,20050},
        { 50,16430},
        { 55,13540},
        { 60,11210},
        { 65,9331}  
	};
#endif

#if defined(CONFIG_MTK_HAFG_20)
/* T0 -10C */
BATTERY_PROFILE_STRUCT battery_profile_t0[] = {
	{0, 4048},
	{2, 4008},
	{4, 3989},
	{5, 3977},
	{7, 3966},
	{9, 3960},
	{11, 3950},
	{12, 3946},
	{14, 3938},
	{16, 3932},
	{18, 3926},
	{19, 3918},
	{21, 3910},
	{23, 3901},
	{25, 3894},
	{26, 3885},
	{28, 3874},
	{30, 3866},
	{32, 3856},
	{33, 3846},
	{35, 3838},
	{37, 3830},
	{39, 3823},
	{40, 3817},
	{42, 3814},
	{44, 3808},
	{46, 3806},
	{47, 3803},
	{49, 3801},
	{51, 3798},
	{53, 3795},
	{54, 3796},
	{56, 3795},
	{58, 3792},
	{60, 3792},
	{61, 3790},
	{63, 3789},
	{65, 3787},
	{67, 3785},
	{68, 3783},
	{70, 3781},
	{72, 3776},
	{74, 3772},
	{75, 3767},
	{77, 3763},
	{79, 3758},
	{81, 3751},
	{82, 3742},
	{84, 3734},
	{86, 3725},
	{88, 3719},
	{90, 3715},
	{91, 3712},
	{93, 3707},
	{95, 3702},
	{97, 3696},
	{98, 3678},
	{100, 3647},
	{101, 3612},
	{102, 3575},
	{103, 3537},
	{103, 3502},
	{104, 3472},
	{104, 3443},
	{104, 3419},
	{105, 3395},
	{105, 3373},
	{105, 3357},
	{105, 3341},
	{105, 3328},
	{105, 3317},
	{105, 3307},
	{105, 3300},
	{105, 3293},
	{105, 3288},
	{105, 3283},
	{105, 3275},
	{105, 3271},
	{105, 3267},
	{105, 3260},
	{106, 3256},
	{106, 3251},
	{106, 3243},
	{106, 3239},
	{106, 3233},
	{106, 3225},
	{106, 3218},
	{106, 3214},
	{106, 3209},
	{106, 3202},
	{106, 3196},
	{106, 3185},
	{106, 3171},
	{106, 3157},
	{106, 3142},
	{106, 3125},
	{106, 3114},
	{106, 3095},
	{106, 3095},
	{106, 3095}
};

/* T1 0C */
BATTERY_PROFILE_STRUCT battery_profile_t1[] = {
	{0, 4048},
	{2, 4008},
	{3, 3989},
	{5, 3977},
	{7, 3966},
	{8, 3960},
	{10, 3956},
	{11, 3951},
	{13, 3948},
	{15, 3941},
	{16, 3935},
	{18, 3928},
	{20, 3922},
	{21, 3914},
	{23, 3906},
	{25, 3898},
	{26, 3892},
	{28, 3882},
	{29, 3872},
	{31, 3860},
	{33, 3849},
	{34, 3839},
	{36, 3831},
	{38, 3824},
	{39, 3818},
	{41, 3815},
	{43, 3808},
	{44, 3805},
	{46, 3803},
	{47, 3798},
	{49, 3796},
	{51, 3793},
	{52, 3792},
	{54, 3790},
	{56, 3790},
	{57, 3788},
	{59, 3788},
	{60, 3787},
	{62, 3787},
	{64, 3785},
	{65, 3785},
	{67, 3784},
	{69, 3782},
	{70, 3779},
	{72, 3777},
	{74, 3774},
	{75, 3769},
	{77, 3766},
	{79, 3762},
	{80, 3756},
	{82, 3748},
	{83, 3742},
	{85, 3734},
	{87, 3724},
	{88, 3714},
	{90, 3708},
	{92, 3703},
	{93, 3701},
	{95, 3699},
	{97, 3696},
	{98, 3689},
	{100, 3662},
	{101, 3601},
	{103, 3533},
	{104, 3475},
	{104, 3418},
	{105, 3363},
	{105, 3315},
	{105, 3270},
	{105, 3238},
	{105, 3208},
	{105, 3191},
	{106, 3172},
	{106, 3159},
	{106, 3150},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137},
	{106, 3137}
};

/* T2 25C */
BATTERY_PROFILE_STRUCT battery_profile_t2[] = {
	{0, 4165},
	{1, 4149},
	{3, 4136},
	{4, 4121},
	{6, 4110},
	{7, 4098},
	{8, 4086},
	{10, 4081},
	{11, 4077},
	{13, 4067},
	{14, 4047},
	{15, 4025},
	{17, 4006},
	{18, 3993},
	{19, 3983},
	{21, 3975},
	{22, 3971},
	{24, 3968},
	{25, 3964},
	{26, 3958},
	{28, 3949},
	{29, 3943},
	{31, 3934},
	{32, 3928},
	{33, 3920},
	{35, 3913},
	{36, 3906},
	{38, 3898},
	{39, 3890},
	{40, 3878},
	{42, 3865},
	{43, 3853},
	{44, 3843},
	{46, 3836},
	{47, 3829},
	{49, 3824},
	{50, 3820},
	{51, 3814},
	{53, 3812},
	{54, 3807},
	{56, 3803},
	{57, 3801},
	{58, 3796},
	{60, 3794},
	{61, 3791},
	{62, 3789},
	{64, 3786},
	{65, 3784},
	{67, 3782},
	{68, 3781},
	{69, 3779},
	{71, 3779},
	{72, 3777},
	{74, 3775},
	{75, 3772},
	{76, 3769},
	{78, 3765},
	{79, 3761},
	{81, 3757},
	{82, 3752},
	{83, 3747},
	{85, 3741},
	{86, 3733},
	{87, 3724},
	{89, 3717},
	{90, 3706},
	{92, 3697},
	{93, 3695},
	{94, 3694},
	{96, 3692},
	{97, 3690},
	{99, 3684},
	{100, 3651},
	{101, 3587},
	{103, 3498},
	{104, 3347},
	{105, 3207},
	{105, 3164},
	{105, 3128},
	{105, 3087},
	{105, 3063},
	{105, 3041},
	{105, 3029},
	{105, 3026},
	{105, 3023},
	{105, 3005},
	{105, 2998},
	{106, 2992},
	{106, 2981},
	{106, 2973},
	{106, 2974},
	{106, 2975},
	{106, 2960},
	{106, 2950},
	{106, 2949},
	{106, 2947},
	{106, 2944},
	{106, 2939},
	{106, 2936},
	{106, 2931}
};

/* T3 50C */
BATTERY_PROFILE_STRUCT battery_profile_t3[] = {
	{0, 4181},
	{1, 4167},
	{3, 4152},
	{4, 4139},
	{5, 4127},
	{7, 4114},
	{8, 4103},
	{10, 4090},
	{11, 4078},
	{12, 4067},
	{14, 4056},
	{15, 4049},
	{16, 4036},
	{18, 4022},
	{19, 4010},
	{20, 4001},
	{22, 3995},
	{23, 3986},
	{25, 3977},
	{26, 3969},
	{27, 3959},
	{29, 3952},
	{30, 3943},
	{31, 3935},
	{33, 3929},
	{34, 3920},
	{35, 3913},
	{37, 3906},
	{38, 3899},
	{40, 3893},
	{41, 3887},
	{42, 3879},
	{44, 3867},
	{45, 3851},
	{46, 3840},
	{48, 3833},
	{49, 3827},
	{50, 3820},
	{52, 3816},
	{53, 3812},
	{55, 3808},
	{56, 3803},
	{57, 3800},
	{59, 3797},
	{60, 3794},
	{61, 3791},
	{63, 3787},
	{64, 3785},
	{65, 3782},
	{67, 3779},
	{68, 3778},
	{70, 3776},
	{71, 3775},
	{72, 3772},
	{74, 3767},
	{75, 3759},
	{76, 3753},
	{78, 3751},
	{79, 3746},
	{81, 3742},
	{82, 3737},
	{83, 3732},
	{85, 3729},
	{86, 3724},
	{87, 3715},
	{89, 3708},
	{90, 3699},
	{91, 3689},
	{93, 3681},
	{94, 3680},
	{95, 3680},
	{97, 3678},
	{98, 3676},
	{100, 3664},
	{101, 3619},
	{102, 3553},
	{104, 3454},
	{105, 3279},
	{106, 3141},
	{106, 3081},
	{106, 3038},
	{106, 3012},
	{106, 2982},
	{106, 2976},
	{106, 2956},
	{106, 2947},
	{106, 2942},
	{106, 2936},
	{106, 2939},
	{106, 2926},
	{106, 2925},
	{106, 2922},
	{106, 2918},
	{106, 2910},
	{106, 2904},
	{106, 2897},
	{106, 2891},
	{106, 2881},
	{106, 2873},
	{106, 2876}
};
#else
/* T0 -10C */
BATTERY_PROFILE_STRUCT battery_profile_t0[] = {
{	0 	,	4374 	},
{	1 	,	4338 	},
{	3 	,	4304 	},
{	4 	,	4276 	},
{	6 	,	4253 	},
{	7 	,	4235 	},
{	8 	,	4218 	},
{	10 	,	4202 	},
{	11 	,	4187 	},
{	12 	,	4173 	},
{	14 	,	4158 	},
{	15 	,	4143 	},
{	17 	,	4129 	},
{	18 	,	4115 	},
{	19 	,	4105 	},
{	21 	,	4095 	},
{	22 	,	4084 	},
{	23 	,	4068 	},
{	25 	,	4045 	},
{	26 	,	4017 	},
{	28 	,	3994 	},
{	29 	,	3976 	},
{	30 	,	3963 	},
{	32 	,	3952 	},
{	33 	,	3942 	},
{	35 	,	3932 	},
{	36 	,	3922 	},
{	37 	,	3914 	},
{	39 	,	3905 	},
{	40 	,	3897 	},
{	41 	,	3889 	},
{	43 	,	3880 	},
{	44 	,	3872 	},
{	46 	,	3863 	},
{	47 	,	3855 	},
{	48 	,	3847 	},
{	50 	,	3840 	},
{	51 	,	3834 	},
{	53 	,	3828 	},
{	54 	,	3823 	},
{	55 	,	3817 	},
{	57 	,	3811 	},
{	58 	,	3807 	},
{	59 	,	3803 	},
{	61 	,	3800 	},
{	62 	,	3798 	},
{	64 	,	3795 	},
{	65 	,	3793 	},
{	66 	,	3790 	},
{	68 	,	3787 	},
{	69 	,	3784 	},
{	70 	,	3781 	},
{	72 	,	3778 	},
{	73 	,	3774 	},
{	75 	,	3770 	},
{	76 	,	3766 	},
{	77 	,	3761 	},
{	79 	,	3756 	},
{	80 	,	3751 	},
{	82 	,	3745 	},
{	83 	,	3739 	},
{	84 	,	3733 	},
{	86 	,	3728 	},
{	87 	,	3723 	},
{	88 	,	3719 	},
{	90 	,	3714 	},
{	91 	,	3709 	},
{	93 	,	3702 	},
{	94 	,	3691 	},
{	95 	,	3673 	},
{	97 	,	3640 	},
{	98 	,	3584 	},
{	99 	,	3482 	},
{	100 	,	3262 	},
{	100 	,	3262 	},
{	100 	,	3262 	},
{	100 	,	3262 	},
{	100 	,	3262 	},
{	100 	,	3262 	}
};

/* T1 0C */
BATTERY_PROFILE_STRUCT battery_profile_t1[] = {
{	0 	,	4378 	},
{	1 	,	4340 	},
{	3 	,	4306 	},
{	4 	,	4283 	},
{	6 	,	4265 	},
{	7 	,	4248 	},
{	8 	,	4231 	},
{	10 	,	4217 	},
{	11 	,	4203 	},
{	12 	,	4189 	},
{	14 	,	4175 	},
{	15 	,	4161 	},
{	17 	,	4147 	},
{	18 	,	4133 	},
{	19 	,	4120 	},
{	21 	,	4107 	},
{	22 	,	4097 	},
{	24 	,	4089 	},
{	25 	,	4082 	},
{	26 	,	4067 	},
{	28 	,	4038 	},
{	29 	,	4009 	},
{	30 	,	3990 	},
{	32 	,	3976 	},
{	33 	,	3965 	},
{	35 	,	3955 	},
{	36 	,	3945 	},
{	37 	,	3935 	},
{	39 	,	3927 	},
{	40 	,	3919 	},
{	41 	,	3907 	},
{	43 	,	3897 	},
{	44 	,	3888 	},
{	46 	,	3879 	},
{	47 	,	3871 	},
{	48 	,	3863 	},
{	50 	,	3857 	},
{	51 	,	3850 	},
{	53 	,	3844 	},
{	54 	,	3838 	},
{	55 	,	3832 	},
{	57 	,	3826 	},
{	58 	,	3822 	},
{	59 	,	3818 	},
{	61 	,	3813 	},
{	62 	,	3809 	},
{	64 	,	3803 	},
{	65 	,	3800 	},
{	66 	,	3795 	},
{	68 	,	3792 	},
{	69 	,	3789 	},
{	71 	,	3786 	},
{	72 	,	3783 	},
{	73 	,	3780 	},
{	75 	,	3777 	},
{	76 	,	3774 	},
{	77 	,	3769 	},
{	79 	,	3764 	},
{	80 	,	3759 	},
{	82 	,	3752 	},
{	83 	,	3745 	},
{	84 	,	3737 	},
{	86 	,	3728 	},
{	87 	,	3720 	},
{	88 	,	3715 	},
{	90 	,	3712 	},
{	91 	,	3710 	},
{	93 	,	3707 	},
{	94 	,	3703 	},
{	95 	,	3692 	},
{	97 	,	3656 	},
{	98 	,	3587 	},
{	100 	,	3477 	},
{	100 	,	3251 	},
{	100 	,	3251 	},
{	100 	,	3251 	},
{	100 	,	3251 	},
{	100 	,	3251 	},
{	100 	,	3251 	}
};

/* T2 25C */
BATTERY_PROFILE_STRUCT battery_profile_t2[] = {
{	0 	,	4383 	},
{	1 	,	4365 	},
{	3 	,	4351 	},
{	4 	,	4335 	},
{	5 	,	4319 	},
{	6 	,	4303 	},
{	8 	,	4287 	},
{	9 	,	4273 	},
{	10 	,	4258 	},
{	12 	,	4243 	},
{	13 	,	4229 	},
{	14 	,	4215 	},
{	15 	,	4200 	},
{	17 	,	4186 	},
{	18 	,	4172 	},
{	19 	,	4159 	},
{	21 	,	4145 	},
{	22 	,	4131 	},
{	23 	,	4117 	},
{	24 	,	4103 	},
{	26 	,	4090 	},
{	27 	,	4078 	},
{	28 	,	4070 	},
{	30 	,	4064 	},
{	31 	,	4051 	},
{	32 	,	4026 	},
{	33 	,	4002 	},
{	35 	,	3986 	},
{	36 	,	3975 	},
{	37 	,	3967 	},
{	39 	,	3961 	},
{	40 	,	3956 	},
{	41 	,	3947 	},
{	42 	,	3934 	},
{	44 	,	3919 	},
{	45 	,	3904 	},
{	46 	,	3892 	},
{	48 	,	3881 	},
{	49 	,	3871 	},
{	50 	,	3863 	},
{	51 	,	3855 	},
{	53 	,	3848 	},
{	54 	,	3841 	},
{	55 	,	3835 	},
{	57 	,	3829 	},
{	58 	,	3823 	},
{	59 	,	3818 	},
{	60 	,	3813 	},
{	62 	,	3808 	},
{	63 	,	3804 	},
{	64 	,	3799 	},
{	66 	,	3795 	},
{	67 	,	3791 	},
{	68 	,	3787 	},
{	69 	,	3783 	},
{	71 	,	3779 	},
{	72 	,	3775 	},
{	73 	,	3772 	},
{	75 	,	3768 	},
{	76 	,	3763 	},
{	77 	,	3758 	},
{	78 	,	3753 	},
{	80 	,	3748 	},
{	81 	,	3743 	},
{	82 	,	3738 	},
{	84 	,	3731 	},
{	85 	,	3721 	},
{	86 	,	3713 	},
{	87 	,	3702 	},
{	89 	,	3693 	},
{	90 	,	3690 	},
{	91 	,	3689 	},
{	93 	,	3687 	},
{	94 	,	3686 	},
{	95 	,	3681 	},
{	96 	,	3655 	},
{	98 	,	3595 	},
{	99 	,	3505 	},
{	100 	,	3358 	}
};

/* T3 50C */
BATTERY_PROFILE_STRUCT battery_profile_t3[] = {
{	0 	,	4399 	},
{	1 	,	4381 	},
{	3 	,	4366 	},
{	4 	,	4352 	},
{	5 	,	4337 	},
{	6 	,	4322 	},
{	8 	,	4308 	},
{	9 	,	4293 	},
{	10 	,	4278 	},
{	12 	,	4263 	},
{	13 	,	4248 	},
{	14 	,	4233 	},
{	16 	,	4219 	},
{	17 	,	4204 	},
{	18 	,	4190 	},
{	19 	,	4175 	},
{	21 	,	4160 	},
{	22 	,	4147 	},
{	23 	,	4133 	},
{	25 	,	4119 	},
{	26 	,	4106 	},
{	27 	,	4093 	},
{	28 	,	4079 	},
{	30 	,	4068 	},
{	31 	,	4056 	},
{	32 	,	4046 	},
{	34 	,	4030 	},
{	35 	,	4015 	},
{	36 	,	4007 	},
{	37 	,	3997 	},
{	39 	,	3987 	},
{	40 	,	3976 	},
{	41 	,	3966 	},
{	43 	,	3955 	},
{	44 	,	3942 	},
{	45 	,	3927 	},
{	47 	,	3912 	},
{	48 	,	3899 	},
{	49 	,	3888 	},
{	50 	,	3879 	},
{	52 	,	3870 	},
{	53 	,	3863 	},
{	54 	,	3855 	},
{	56 	,	3849 	},
{	57 	,	3843 	},
{	58 	,	3836 	},
{	59 	,	3831 	},
{	61 	,	3825 	},
{	62 	,	3820 	},
{	63 	,	3815 	},
{	65 	,	3811 	},
{	66 	,	3806 	},
{	67 	,	3802 	},
{	68 	,	3798 	},
{	70 	,	3794 	},
{	71 	,	3789 	},
{	72 	,	3783 	},
{	74 	,	3773 	},
{	75 	,	3765 	},
{	76 	,	3759 	},
{	78 	,	3752 	},
{	79 	,	3746 	},
{	80 	,	3740 	},
{	81 	,	3735 	},
{	83 	,	3730 	},
{	84 	,	3724 	},
{	85 	,	3714 	},
{	87 	,	3707 	},
{	88 	,	3694 	},
{	89 	,	3685 	},
{	90 	,	3684 	},
{	92 	,	3682 	},
{	93 	,	3681 	},
{	94 	,	3679 	},
{	96 	,	3672 	},
{	97 	,	3633 	},
{	98 	,	3566 	},
{	99 	,	3468 	},
{	100 	,	3291 	}
};
#endif

/* battery profile for actual temperature. The size should be the same as T1, T2 and T3 */
BATTERY_PROFILE_STRUCT battery_profile_temperature[] = {
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0}
};

#if defined(CONFIG_MTK_HAFG_20)
/* T0 -10C */
R_PROFILE_STRUCT r_profile_t0[] = {
	{865, 4048},
	{865, 4008},
	{893, 3989},
	{915, 3977},
	{955, 3966},
	{1023, 3960},
	{1200, 3950},
	{1338, 3946},
	{1375, 3938},
	{1388, 3932},
	{1408, 3926},
	{1420, 3918},
	{1428, 3910},
	{1418, 3901},
	{1428, 3894},
	{1423, 3885},
	{1418, 3874},
	{1425, 3866},
	{1428, 3856},
	{1428, 3846},
	{1425, 3838},
	{1423, 3830},
	{1420, 3823},
	{1415, 3817},
	{1425, 3814},
	{1425, 3808},
	{1450, 3806},
	{1468, 3803},
	{1465, 3801},
	{1483, 3798},
	{1488, 3795},
	{1510, 3796},
	{1515, 3795},
	{1533, 3792},
	{1535, 3792},
	{1548, 3790},
	{1543, 3789},
	{1563, 3787},
	{1588, 3785},
	{1610, 3783},
	{1625, 3781},
	{1640, 3776},
	{1653, 3772},
	{1660, 3767},
	{1680, 3763},
	{1690, 3758},
	{1710, 3751},
	{1733, 3742},
	{1745, 3734},
	{1765, 3725},
	{1788, 3719},
	{1813, 3715},
	{1853, 3712},
	{1905, 3707},
	{1965, 3702},
	{2010, 3696},
	{2080, 3678},
	{2123, 3647},
	{2035, 3612},
	{1943, 3575},
	{1853, 3537},
	{1770, 3502},
	{1685, 3472},
	{1623, 3443},
	{1550, 3419},
	{1493, 3395},
	{1448, 3373},
	{1395, 3357},
	{1368, 3341},
	{1338, 3328},
	{1303, 3317},
	{1298, 3307},
	{1263, 3300},
	{1253, 3293},
	{1260, 3288},
	{1225, 3283},
	{1240, 3275},
	{1198, 3271},
	{1215, 3267},
	{1198, 3260},
	{1200, 3256},
	{1218, 3251},
	{1228, 3243},
	{1138, 3239},
	{1230, 3233},
	{1243, 3225},
	{1155, 3218},
	{1165, 3214},
	{1045, 3209},
	{1170, 3202},
	{1183, 3196},
	{1340, 3185},
	{1368, 3171},
	{1423, 3157},
	{1455, 3142},
	{1533, 3125},
	{1365, 3114},
	{1653, 3095},
	{1653, 3095},
	{1653, 3095}
};
#else
/* T0 -10C */
R_PROFILE_STRUCT r_profile_t0[] = {
{	970 	,	4374 	},
{	970 	,	4338 	},
{	1006 	,	4304 	},
{	1045 	,	4276 	},
{	1068 	,	4253 	},
{	1085 	,	4235 	},
{	1090 	,	4218 	},
{	1099 	,	4202 	},
{	1102 	,	4187 	},
{	1107 	,	4173 	},
{	1105 	,	4158 	},
{	1101 	,	4143 	},
{	1098 	,	4129 	},
{	1095 	,	4115 	},
{	1102 	,	4105 	},
{	1113 	,	4095 	},
{	1115 	,	4084 	},
{	1107 	,	4068 	},
{	1081 	,	4045 	},
{	1050 	,	4017 	},
{	1032 	,	3994 	},
{	1020 	,	3976 	},
{	1020 	,	3963 	},
{	1017 	,	3952 	},
{	1015 	,	3942 	},
{	1014 	,	3932 	},
{	1012 	,	3922 	},
{	1009 	,	3914 	},
{	1003 	,	3905 	},
{	1000 	,	3897 	},
{	994 	,	3889 	},
{	995 	,	3880 	},
{	988 	,	3872 	},
{	987 	,	3863 	},
{	986 	,	3855 	},
{	986 	,	3847 	},
{	988 	,	3840 	},
{	994 	,	3834 	},
{	997 	,	3828 	},
{	1000 	,	3823 	},
{	1000 	,	3817 	},
{	1008 	,	3811 	},
{	1014 	,	3807 	},
{	1022 	,	3803 	},
{	1039 	,	3800 	},
{	1060 	,	3798 	},
{	1077 	,	3795 	},
{	1098 	,	3793 	},
{	1118 	,	3790 	},
{	1140 	,	3787 	},
{	1163 	,	3784 	},
{	1189 	,	3781 	},
{	1216 	,	3778 	},
{	1240 	,	3774 	},
{	1268 	,	3770 	},
{	1293 	,	3766 	},
{	1318 	,	3761 	},
{	1342 	,	3756 	},
{	1367 	,	3751 	},
{	1394 	,	3745 	},
{	1420 	,	3739 	},
{	1449 	,	3733 	},
{	1480 	,	3728 	},
{	1518 	,	3723 	},
{	1558 	,	3719 	},
{	1608 	,	3714 	},
{	1660 	,	3709 	},
{	1716 	,	3702 	},
{	1773 	,	3691 	},
{	1836 	,	3673 	},
{	1885 	,	3640 	},
{	1938 	,	3584 	},
{	2009 	,	3482 	},
{	2863 	,	3262 	},
{	2863 	,	3262 	},
{	2863 	,	3262 	},
{	2863 	,	3262 	},
{	2863 	,	3262 	},
{	2863 	,	3262 	}
};
#endif

/* T1 0C */
R_PROFILE_STRUCT r_profile_t1[] = {
{	397 	,	4378 	},
{	397 	,	4340 	},
{	436 	,	4306 	},
{	462 	,	4283 	},
{	468 	,	4265 	},
{	468 	,	4248 	},
{	482 	,	4231 	},
{	482 	,	4217 	},
{	490 	,	4203 	},
{	490 	,	4189 	},
{	493 	,	4175 	},
{	491 	,	4161 	},
{	494 	,	4147 	},
{	498 	,	4133 	},
{	496 	,	4120 	},
{	498 	,	4107 	},
{	510 	,	4097 	},
{	526 	,	4089 	},
{	540 	,	4082 	},
{	535 	,	4067 	},
{	507 	,	4038 	},
{	493 	,	4009 	},
{	491 	,	3990 	},
{	488 	,	3976 	},
{	490 	,	3965 	},
{	487 	,	3955 	},
{	485 	,	3945 	},
{	477 	,	3935 	},
{	473 	,	3927 	},
{	473 	,	3919 	},
{	460 	,	3907 	},
{	453 	,	3897 	},
{	453 	,	3888 	},
{	450 	,	3879 	},
{	450 	,	3871 	},
{	448 	,	3863 	},
{	449 	,	3857 	},
{	453 	,	3850 	},
{	456 	,	3844 	},
{	457 	,	3838 	},
{	457 	,	3832 	},
{	459 	,	3826 	},
{	464 	,	3822 	},
{	465 	,	3818 	},
{	468 	,	3813 	},
{	471 	,	3809 	},
{	470 	,	3803 	},
{	473 	,	3800 	},
{	473 	,	3795 	},
{	478 	,	3792 	},
{	481 	,	3789 	},
{	490 	,	3786 	},
{	496 	,	3783 	},
{	505 	,	3780 	},
{	512 	,	3777 	},
{	521 	,	3774 	},
{	529 	,	3769 	},
{	543 	,	3764 	},
{	552 	,	3759 	},
{	563 	,	3752 	},
{	574 	,	3745 	},
{	581 	,	3737 	},
{	589 	,	3728 	},
{	602 	,	3720 	},
{	615 	,	3715 	},
{	639 	,	3712 	},
{	669 	,	3710 	},
{	699 	,	3707 	},
{	739 	,	3703 	},
{	781 	,	3692 	},
{	808 	,	3656 	},
{	834 	,	3587 	},
{	896 	,	3477 	},
{	1119 	,	3251 	},
{	1119 	,	3251 	},
{	1119 	,	3251 	},
{	1119 	,	3251 	},
{	1119 	,	3251 	},
{	1119 	,	3251 	}
};

/* T2 25C */
R_PROFILE_STRUCT r_profile_t2[] = {
{	170 	,	4383 	},
{	170 	,	4365 	},
{	172 	,	4351 	},
{	172 	,	4335 	},
{	170 	,	4319 	},
{	170 	,	4303 	},
{	171 	,	4287 	},
{	172 	,	4273 	},
{	172 	,	4258 	},
{	171 	,	4243 	},
{	172 	,	4229 	},
{	174 	,	4215 	},
{	175 	,	4200 	},
{	175 	,	4186 	},
{	186 	,	4172 	},
{	194 	,	4159 	},
{	194 	,	4145 	},
{	196 	,	4131 	},
{	197 	,	4117 	},
{	199 	,	4103 	},
{	201 	,	4090 	},
{	204 	,	4078 	},
{	212 	,	4070 	},
{	223 	,	4064 	},
{	224 	,	4051 	},
{	216 	,	4026 	},
{	212 	,	4002 	},
{	213 	,	3986 	},
{	214 	,	3975 	},
{	223 	,	3967 	},
{	226 	,	3961 	},
{	229 	,	3956 	},
{	219 	,	3947 	},
{	206 	,	3934 	},
{	193 	,	3919 	},
{	182 	,	3904 	},
{	174 	,	3892 	},
{	168 	,	3881 	},
{	165 	,	3871 	},
{	164 	,	3863 	},
{	162 	,	3855 	},
{	162 	,	3848 	},
{	163 	,	3841 	},
{	162 	,	3835 	},
{	163 	,	3829 	},
{	164 	,	3823 	},
{	164 	,	3818 	},
{	167 	,	3813 	},
{	167 	,	3808 	},
{	170 	,	3804 	},
{	169 	,	3799 	},
{	171 	,	3795 	},
{	172 	,	3791 	},
{	173 	,	3787 	},
{	173 	,	3783 	},
{	178 	,	3779 	},
{	184 	,	3775 	},
{	182 	,	3772 	},
{	181 	,	3768 	},
{	178 	,	3763 	},
{	176 	,	3758 	},
{	173 	,	3753 	},
{	175 	,	3748 	},
{	175 	,	3743 	},
{	178 	,	3738 	},
{	179 	,	3731 	},
{	177 	,	3721 	},
{	181 	,	3713 	},
{	179 	,	3702 	},
{	175 	,	3693 	},
{	178 	,	3690 	},
{	183 	,	3689 	},
{	191 	,	3687 	},
{	201 	,	3686 	},
{	209 	,	3681 	},
{	200 	,	3655 	},
{	200 	,	3595 	},
{	212 	,	3505 	},
{	249 	,	3358 	}
};

/* T3 50C */
R_PROFILE_STRUCT r_profile_t3[] = {
{	109 	,	4399 	},
{	109 	,	4381 	},
{	107 	,	4366 	},
{	108 	,	4352 	},
{	107 	,	4337 	},
{	107 	,	4322 	},
{	110 	,	4308 	},
{	108 	,	4293 	},
{	110 	,	4278 	},
{	108 	,	4263 	},
{	109 	,	4248 	},
{	110 	,	4233 	},
{	110 	,	4219 	},
{	113 	,	4204 	},
{	110 	,	4190 	},
{	112 	,	4175 	},
{	111 	,	4160 	},
{	113 	,	4147 	},
{	116 	,	4133 	},
{	116 	,	4119 	},
{	118 	,	4106 	},
{	121 	,	4093 	},
{	119 	,	4079 	},
{	126 	,	4068 	},
{	126 	,	4056 	},
{	130 	,	4046 	},
{	127 	,	4030 	},
{	130 	,	4015 	},
{	136 	,	4007 	},
{	135 	,	3997 	},
{	138 	,	3987 	},
{	138 	,	3976 	},
{	144 	,	3966 	},
{	144 	,	3955 	},
{	139 	,	3942 	},
{	132 	,	3927 	},
{	121 	,	3912 	},
{	113 	,	3899 	},
{	108 	,	3888 	},
{	110 	,	3879 	},
{	108 	,	3870 	},
{	110 	,	3863 	},
{	109 	,	3855 	},
{	112 	,	3849 	},
{	113 	,	3843 	},
{	112 	,	3836 	},
{	113 	,	3831 	},
{	113 	,	3825 	},
{	115 	,	3820 	},
{	118 	,	3815 	},
{	121 	,	3811 	},
{	123 	,	3806 	},
{	122 	,	3802 	},
{	123 	,	3798 	},
{	126 	,	3794 	},
{	124 	,	3789 	},
{	121 	,	3783 	},
{	113 	,	3773 	},
{	113 	,	3765 	},
{	112 	,	3759 	},
{	113 	,	3752 	},
{	112 	,	3746 	},
{	113 	,	3740 	},
{	115 	,	3735 	},
{	116 	,	3730 	},
{	115 	,	3724 	},
{	112 	,	3714 	},
{	116 	,	3707 	},
{	110 	,	3694 	},
{	107 	,	3685 	},
{	110 	,	3684 	},
{	113 	,	3682 	},
{	119 	,	3681 	},
{	126 	,	3679 	},
{	132 	,	3672 	},
{	120 	,	3633 	},
{	126 	,	3566 	},
{	133 	,	3468 	},
{	161 	,	3291 	}
};

/* r-table profile for actual temperature. The size should be the same as T1, T2 and T3 */
R_PROFILE_STRUCT r_profile_temperature[] = {
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0},
	{0, 0}
};

/* ============================================================
// function prototype
// ============================================================*/
int fgauge_get_saddles(void);
BATTERY_PROFILE_STRUCT_P fgauge_get_profile(unsigned int temperature);

int fgauge_get_saddles_r_table(void);
R_PROFILE_STRUCT_P fgauge_get_profile_r_table(unsigned int temperature);

#endif	/*#ifndef _CUST_BATTERY_METER_TABLE_H*/

