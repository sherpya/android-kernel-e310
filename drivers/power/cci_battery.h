/* Copyright (c) 2009, Code Aurora Forum. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Code Aurora nor
 *       the names of its contributors may be used to endorse or promote
 *       products derived from this software without specific prior written
 *       permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NON-INFRINGEMENT ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#define FAKE_CAPACITY_INIT -9999
#ifndef CONFIG_HAS_EARLYSUSPEND
#define CONFIG_HAS_EARLYSUSPEND
#endif


extern void cci_android_charger_usb_change (int plugin, int type);

extern int IsUSBPlugged(void);
/********************************************************************************
 * Hardware ID and Project ID
 *******************************************************************************/
int vbatt;
int batt_capacity;
enum {
    HARDWARE_ID_EMU,
    HARDWARE_ID_EVT0,
    HARDWARE_ID_EVT1,
    HARDWARE_ID_EVT2,
    HARDWARE_ID_DVT1,
    HARDWARE_ID_DVT2,
    HARDWARE_ID_DVT3,
    HARDWARE_ID_PVT,
};   
enum {                                                                                                                           
    BATTERY_K4_SMP,
    BATTERY_K4_MEI,
};

/********************************************************************************
 * Current mapping table
 *******************************************************************************/



/*
 *
 * K4 SMP Battery table
 *
 */

// 100mA
static int battery_capacity_down_table_0_100_k4[] __initdata =
{
    /* 0%   5%   10%   15%   20%   25%   30%   35%   40%   45%   50%
     55%   60%   65%   70%   75%   80%   85%   90%   95%  100%    */
    3400, 3520, 3581, 3621, 3655, 3678, 3693, 3703, 3713, 3723, 3738,
    3757, 3778, 3804, 3834, 3865, 3893, 3929, 3995, 4041, 4200,
    3520, 3581, 3621, 3655, 3678, 3693, 3703, 3713, 3723, 3738, 3757,
    3778, 3804, 3834, 3865, 3893, 3929, 3995, 4041, 4120
};
static int battery_capacity_down_table_25_100_k4[] __initdata =
{
    3400, 3628, 3654, 3692, 3719, 3734, 3740, 3745, 3753, 3766, 3782,
    3803, 3838, 3872, 3903, 3934, 3961, 4020, 4056, 4103, 4200,
    3628, 3654, 3692, 3719, 3734, 3740, 3745, 3753, 3766, 3782, 3803,
    3838, 3872, 3903, 3934, 3961, 4020, 4056, 4103, 4120
};
static int battery_capacity_down_table_45_100_k4[] __initdata =
{
    3400, 3645, 3657, 3692, 3716, 3732, 3744, 3752, 3761, 3775, 3791,
    3816, 3855, 3884, 3914, 3949, 3983, 4029, 4070, 4116, 4200,
    3645, 3657, 3692, 3716, 3732, 3744, 3752, 3761, 3775, 3791, 3816, 
    3855, 3884, 3914, 3949, 3983, 4029, 4070, 4116, 4120
};

// 200mA
static int battery_capacity_down_table_0_200_k4[] __initdata =
{
    3405, 3481, 3530, 3565, 3593, 3616, 3634, 3649, 3662, 3678, 3695,
    3716, 3737, 3762, 3788, 3816, 3847, 3889, 3951, 4000, 4200,
    3481, 3530, 3565, 3593, 3616, 3634, 3649, 3662, 3678, 3695, 3716,
    3737, 3762, 3788, 3816, 3847, 3889, 3951, 4000, 4120

};
static int battery_capacity_down_table_25_200_k4[] __initdata =
{
    3400, 3595, 3629, 3668, 3693, 3706, 3715, 3722, 3732, 3745, 3763,
    3784, 3811, 3845, 3879, 3910, 3941, 4001, 4037, 4086, 4200,
    3595, 3629, 3668, 3693, 3706, 3715, 3722, 3732, 3745, 3763, 3784,
    3811, 3845, 3879, 3910, 3941, 4001, 4037, 4086, 4120
};
static int battery_capacity_down_table_45_200_k4[] __initdata =
{
    3400, 3623, 3638, 3675, 3698, 3715, 3724, 3733, 3744, 3759, 3776,
    3800, 3835, 3866, 3899, 3933, 3969, 4013, 4055, 4102, 4200,
    3623, 3638, 3675, 3698, 3715, 3724, 3733, 3744, 3759, 3776, 3800,
    3835, 3866, 3899, 3933, 3969, 4013, 4055, 4102, 4120
};

// 300mA
static int battery_capacity_down_table_0_300_k4[] __initdata =
{
    3400, 3458, 3495, 3525, 3548, 3569, 3587, 3603, 3618, 3635, 3653,
    3674, 3696, 3720, 3745, 3773, 3804, 3846, 3905, 3957, 4200,
    3458, 3495, 3525, 3548, 3569, 3587, 3603, 3618, 3635, 3653, 3674,
    3696, 3720, 3745, 3773, 3804, 3846, 3905, 3957, 4120
};
static int battery_capacity_down_table_25_300_k4[] __initdata =
{
    3400, 3560, 3601, 3639, 3664, 3678, 3688, 3697, 3708, 3722, 3740,
    3761, 3787, 3818, 3851, 3882, 3915, 3973, 4013, 4062, 4200,
    3560, 3601, 3639, 3664, 3678, 3688, 3697, 3708, 3722, 3740, 3761,
    3787, 3818, 3851, 3882, 3915, 3973, 4013, 4062, 4120
};
static int battery_capacity_down_table_45_300_k4[] __initdata =
{
    3400, 3601, 3620, 3657, 3680, 3696, 3705, 3714, 3726, 3741, 3759,
    3783, 3815, 3846, 3878, 3913, 3950, 3994, 4037, 4084, 4200,
    3601, 3620, 3657, 3680, 3696, 3705, 3714, 3726, 3741, 3759, 3783,
    3815, 3846, 3878, 3913, 3950, 3994, 4037, 4084, 4120
};

// 400mA
static int battery_capacity_down_table_0_400_k4[] __initdata =
{
    3400, 3434, 3460, 3484, 3503, 3522, 3539, 3556, 3573, 3591, 3611,
    3632, 3654, 3678, 3702, 3730, 3761, 3803, 3859, 3914, 4200,
    3434, 3460, 3484, 3503, 3522, 3539, 3556, 3573, 3591, 3611, 3632,
    3654, 3678, 3702, 3730, 3761, 3803, 3859, 3914, 4120
};
static int battery_capacity_down_table_25_400_k4[] __initdata =
{
    3400, 3525, 3572, 3609, 3634, 3649, 3660, 3671, 3683, 3698, 3716,
    3738, 3763, 3791, 3823, 3854, 3889, 3944, 3988, 4037, 4200,
    3525, 3572, 3609, 3634, 3649, 3660, 3671, 3683, 3698, 3716, 3738,
    3763, 3791, 3823, 3854, 3889, 3944, 3988, 4037, 4120
};
static int battery_capacity_down_table_45_400_k4[] __initdata =
{   
    3400, 3579, 3602, 3638, 3662, 3677, 3686, 3695, 3707, 3723, 3742,
    3765, 3794, 3825, 3857, 3893, 3930, 3975, 4018, 4066, 4200,
    3579, 3602, 3638, 3662, 3677, 3686, 3695, 3707, 3723, 3742, 3765,
    3794, 3825, 3857, 3893, 3930, 3975, 4018, 4066, 4120
};

// 500mA
static int battery_capacity_down_table_0_500_k4[] __initdata =
{
    3400, 3426, 3446, 3466, 3483, 3500, 3515, 3531, 3548, 3565, 3584,
    3604, 3625, 3647, 3671, 3699, 3731, 3771, 3821, 3876, 4200,
    3426, 3446, 3466, 3483, 3500, 3515, 3531, 3548, 3565, 3584, 3604,
    3625, 3647, 3671, 3699, 3731, 3771, 3821, 3876, 4120
};
static int battery_capacity_down_table_25_500_k4[] __initdata =
{
    3400, 3510, 3553, 3585, 3608, 3624, 3637, 3649, 3662, 3678, 3697,
    3720, 3745, 3773, 3804, 3835, 3871, 3925, 3969, 4020, 4200,
    3510, 3553, 3585, 3608, 3624, 3637, 3649, 3662, 3678, 3697, 3720,
    3745, 3773, 3804, 3835, 3871, 3925, 3969, 4020, 4120
};
static int battery_capacity_down_table_45_500_k4[] __initdata =
{
    3400, 3561, 3586, 3621, 3644, 3658, 3668, 3678, 3690, 3707, 3727,
    3750, 3777, 3808, 3840, 3876, 3915, 3960, 4004, 4053, 4200,
    3561, 3586, 3621, 3644, 3658, 3668, 3678, 3690, 3707, 3727, 3750,
    3777, 3808, 3840, 3876, 3915, 3960, 4004, 4053, 4120
};

// 600mA
static int battery_capacity_down_table_0_600_k4[] __initdata =
{
    3400, 3417, 3432, 3447, 3462, 3477, 3491, 3506, 3522, 3538, 3556,
    3576, 3595, 3615, 3639, 3668, 3700, 3738, 3782, 3837, 4200,
    3417, 3432, 3447, 3462, 3477, 3491, 3506, 3522, 3538, 3556, 3576,
    3595, 3615, 3639, 3668, 3700, 3738, 3782, 3837, 4120
};
static int battery_capacity_down_table_25_600_k4[] __initdata =
{
    3400, 3495, 3533, 3561, 3582, 3598, 3613, 3627, 3641, 3658, 3678,
    3701, 3727, 3754, 3784, 3815, 3852, 3906, 3950, 4002, 4200,
    3495, 3533, 3561, 3582, 3598, 3613, 3627, 3641, 3658, 3678, 3701,
    3727, 3754, 3784, 3815, 3852, 3906, 3950, 4002, 4120
};
static int battery_capacity_down_table_45_600_k4[] __initdata =
{
    3400, 3542, 3569, 3603, 3625, 3639, 3650, 3661, 3673, 3690, 3711,
    3734, 3760, 3790, 3822, 3859, 3899, 3945, 3990, 4040, 4200,
    3542, 3569, 3603, 3625, 3639, 3650, 3661, 3673, 3690, 3711, 3734,
    3760, 3790, 3822, 3859, 3899, 3945, 3990, 4040, 4120

};
// 700mA
static int battery_capacity_down_table_0_700_k4[] __initdata =
{
    3400, 3416, 3427, 3439, 3451, 3463, 3475, 3487, 3500, 3513, 3527,
    3543, 3557, 3573, 3590, 3609, 3633, 3665, 3711, 3779, 4200,
    3416, 3427, 3439, 3451, 3463, 3475, 3487, 3500, 3513, 3527, 3543,
    3557, 3573, 3590, 3609, 3633, 3665, 3711, 3779, 4120
};
static int battery_capacity_down_table_25_700_k4[] __initdata =
{
    3400, 3472, 3507, 3532, 3554, 3571, 3587, 3602, 3617, 3634, 3654,
    3677, 3703, 3729, 3759, 3790, 3827, 3878, 3923, 3975, 4200,
    3472, 3507, 3532, 3554, 3571, 3587, 3602, 3617, 3634, 3654, 3677,
    3703, 3729, 3759, 3790, 3827, 3878, 3923, 3975, 4120
};
static int battery_capacity_down_table_45_700_k4[] __initdata =
{
    3400, 3520, 3550, 3581, 3602, 3617, 3630, 3642, 3654, 3671, 3692,
    3715, 3741, 3770, 3802, 3838, 3878, 3923, 3969, 4019, 4200,
    3520, 3550, 3581, 3602, 3617, 3630, 3642, 3654, 3671, 3692, 3715,
    3741, 3770, 3802, 3838, 3878, 3923, 3969, 4019, 4120
};



//AC 800ma
static int battery_capacity_up_table_0_ac_800[] __initdata =
{
    3400, 4026, 4060, 4080, 4095, 4105, 4113, 4120, 4126, 4132,
    4137, 4143, 4149, 4155, 4162
};
static int battery_capacity_up_table_0_ac_400[] __initdata =
{
    4162, 4175, 4190, 4200
};

static int battery_capacity_up_table_25_ac_800[] __initdata =
{
    3400, 3861, 3885, 3912, 3939, 3958, 3968, 3980, 3993, 4004,
    4015, 4026, 4039, 4052, 4100
};
static int battery_capacity_up_table_25_ac_400[] __initdata =
{
    4068, 4105, 4150, 4200
};

static int battery_capacity_up_table_45_ac_800[] __initdata =
{
    3400, 3814, 3837, 3860, 3889, 3914, 3925, 3935, 3946, 3958, 
    3971, 3982, 3994, 4010, 4100 
};
static int battery_capacity_up_table_45_ac_400[] __initdata =
{
    4028, 4075, 4134, 4200
};

//USB 400ma
static int battery_capacity_up_table_0_usb_k4[] __initdata =
{
    3400, 3979, 3994, 4009, 4026, 4044, 4067, 4091, 4105, 4100
};
static int battery_capacity_up_table_0_usb_k41[] __initdata =
{
    4156, 4178, 4200
};

static int battery_capacity_up_table_25_usb_k4[] __initdata =
{
    3400, 3802, 3859, 3887, 3906, 3931, 3959, 3996, 4044, 4100 
};
static int battery_capacity_up_table_25_usb_k41[] __initdata =
{
    4100, 4149, 4200
};

static int battery_capacity_up_table_45_usb_k4[] __initdata =
{
    3400, 3765, 3823, 3855, 3870, 3895, 3936, 3982, 4040, 4100
};
static int battery_capacity_up_table_45_usb_k41[] __initdata =
{
    4100, 4157, 4200
};


/*
 *
 * K4 Mei Battery table
 *
 */
// 100mA
static int battery_capacity_down_table_0_100_k4_mei[] __initdata =
{
    /* 0%   5%   10%   15%   20%   25%   30%   35%   40%   45%   50%
     55%   60%   65%   70%   75%   80%   85%   90%   95%  100%    */
    3400, 3520, 3573, 3623, 3658, 3685, 3706, 3720, 3729, 3741, 3758,
    3779, 3803, 3833, 3867, 3904, 3944, 3986, 4030, 4080, 4200,
    3520, 3573, 3623, 3658, 3685, 3706, 3720, 3729, 3741, 3758, 3779,
    3803, 3833, 3867, 3904, 3944, 3986, 4030, 4080, 4120

};
static int battery_capacity_down_table_25_100_k4_mei[] __initdata =
{
    3400, 3648, 3661, 3693, 3723, 3740, 3748, 3758, 3770, 3783, 3800,
    3823, 3858, 3893, 3925, 3958, 3994, 4034, 4076, 4122, 4200,
    3648, 3661, 3693, 3723, 3740, 3748, 3758, 3770, 3783, 3800, 3823,
    3858, 3893, 3925, 3958, 3994, 4034, 4076, 4090, 4120
};
static int battery_capacity_down_table_45_100_k4_mei[] __initdata =
{
    3400, 3651, 3661, 3690, 3718, 3736, 3753, 3764, 3775, 3789, 3806,
    3829, 3869, 3899, 3929, 3961, 3996, 4035, 4077, 4123, 4200,
    3651, 3661, 3690, 3718, 3736, 3753, 3764, 3775, 3789, 3806, 3829,
    3869, 3899, 3929, 3961, 3996, 4035, 4077, 4090, 4120
};
// 200mA
static int battery_capacity_down_table_0_200_k4_mei[] __initdata =
{
    3400, 3492, 3539, 3582, 3615, 3642, 3664, 3680, 3692, 3704, 3720,
    3739, 3762, 3787, 3817, 3852, 3889, 3929, 3972, 4022, 4200,
    3492, 3539, 3582, 3615, 3642, 3664, 3680, 3692, 3704, 3720, 3739,
    3762, 3787, 3817, 3852, 3889, 3929, 3972, 4022, 4120

};
static int battery_capacity_down_table_25_200_k4_mei[] __initdata =
{
    3400, 3615, 3634, 3670, 3701, 3717, 3727, 3736, 3748, 3763, 3780,
    3802, 3834, 3870, 3903, 3938, 3975, 4015, 4058, 4104, 4200,
    3615, 3634, 3670, 3701, 3717, 3727, 3736, 3748, 3763, 3780, 3802,
    3834, 3870, 3903, 3938, 3975, 4015, 4058, 4090, 4120
};
static int battery_capacity_down_table_45_200_k4_mei[] __initdata =
{
    3400, 3636, 3647, 3678, 3706, 3723, 3736, 3748, 3761, 3776, 3793,
    3816, 3851, 3884, 3915, 3948, 3984, 4024, 4066, 4111, 4200,
    3636, 3647, 3678, 3706, 3723, 3736, 3748, 3761, 3776, 3793, 3816,
    3851, 3884, 3915, 3948, 3984, 4024, 4066, 4090, 4120
};

// 300mA
static int battery_capacity_down_table_0_300_k4_mei[] __initdata =
{
    3400, 3467, 3508, 3545, 3576, 3603, 3625, 3643, 3657, 3670, 3686,
    3704, 3726, 3750, 3778, 3809, 3845, 3883, 3926, 3980, 4200,
    3467, 3508, 3545, 3576, 3603, 3625, 3643, 3657, 3670, 3686, 3704,
    3726, 3750, 3778, 3809, 3845, 3883, 3926, 3980, 4120        
};
static int battery_capacity_down_table_25_300_k4_mei[] __initdata =
{
    3400, 3586, 3610, 3649, 3679, 3696, 3706, 3716, 3728, 3743, 3761,
    3784, 3814, 3849, 3883, 3918, 3956, 3996, 4040, 4087, 4200,
    3586, 3610, 3649, 3679, 3696, 3706, 3716, 3728, 3743, 3761, 3784,
    3814, 3849, 3883, 3918, 3956, 3996, 4040, 4087, 4120
};
static int battery_capacity_down_table_45_300_k4_mei[] __initdata =
{
    3400, 3618, 3631, 3663, 3690, 3707, 3720, 3732, 3745, 3760, 3778,
    3801, 3835, 3868, 3899, 3933, 3969, 4009, 4052, 4097, 4200,
    3618, 3631, 3663, 3690, 3707, 3720, 3732, 3745, 3760, 3778, 3801,
    3835, 3868, 3899, 3933, 3969, 4009, 4052, 4097, 4120
};

// 400mA
static int battery_capacity_down_table_0_400_k4_mei[] __initdata =
{
    3400, 3442, 3478, 3509, 3537, 3564, 3587, 3606, 3622, 3635, 3651,
    3669, 3690, 3713, 3738, 3767, 3800, 3836, 3879, 3938, 4200,
    3442, 3478, 3509, 3537, 3564, 3587, 3606, 3622, 3635, 3651, 3669,
    3690, 3713, 3738, 3767, 3800, 3836, 3879, 3938, 4120
};
static int battery_capacity_down_table_25_400_k4_mei[] __initdata =
{
    3400, 3557, 3585, 3627, 3656, 3674, 3686, 3696, 3709, 3724, 3743,
    3765, 3793, 3827, 3862, 3899, 3937, 3978, 4022, 4070, 4200,
    3557, 3585, 3627, 3656, 3674, 3686, 3696, 3709, 3724, 3743, 3765,
    3793, 3827, 3862, 3899, 3937, 3978, 4022, 4070, 4120
};
static int battery_capacity_down_table_45_400_k4_mei[] __initdata =
{
    3413, 3600, 3614, 3648, 3675, 3692, 3703, 3715, 3729, 3745, 3763,
    3786, 3819, 3852, 3884, 3917, 3954, 3995, 4038, 4083, 4200,
    3600, 3614, 3648, 3675, 3692, 3703, 3715, 3729, 3745, 3763, 3786,
    3819, 3852, 3884, 3917, 3954, 3995, 4038, 4083, 4120
};

// 500mA
static int battery_capacity_down_table_0_500_k4_mei[] __initdata =
{
    3400, 3436, 3466, 3493, 3518, 3542, 3562, 3580, 3595, 3609, 3625,
    3642, 3662, 3684, 3709, 3736, 3767, 3801, 3844, 3904, 4200,
    3436, 3466, 3493, 3518, 3542, 3562, 3580, 3595, 3609, 3625, 3642,
    3662, 3684, 3709, 3736, 3767, 3801, 3844, 3904, 4120
};
static int battery_capacity_down_table_25_500_k4_mei[] __initdata =
{
    3400, 3531, 3564, 3605, 3634, 3653, 3666, 3677, 3690, 3706, 3725,
    3748, 3775, 3808, 3843, 3880, 3919, 3960, 4005, 4055, 4200,
    3531, 3564, 3605, 3634, 3653, 3666, 3677, 3690, 3706, 3725, 3748,
    3775, 3808, 3843, 3880, 3919, 3960, 4005, 4055, 4120
};
static int battery_capacity_down_table_45_500_k4_mei[] __initdata =
{
    3400, 3585, 3601, 3636, 3662, 3677, 3689, 3701, 3715, 3731, 3750,
    3773, 3804, 3837, 3869, 3904, 3941, 3981, 4025, 4070, 4200,
    3585, 3601, 3636, 3662, 3677, 3689, 3701, 3715, 3731, 3750, 3773,
    3804, 3837, 3869, 3904, 3941, 3981, 4025, 4070, 4120
};

// 600mA
static int battery_capacity_down_table_0_600_k4_mei[] __initdata =
{
    3400, 3429, 3454, 3477, 3499, 3520, 3538, 3554, 3568, 3583, 3598,
    3616, 3635, 3656, 3680, 3705, 3733, 3767, 3808, 3871, 4200,
    3429, 3454, 3477, 3499, 3520, 3538, 3554, 3568, 3583, 3598, 3616,
    3635, 3656, 3680, 3705, 3733, 3767, 3808, 3871, 4120        
};
static int battery_capacity_down_table_25_600_k4_mei[] __initdata =
{
    3400, 3505, 3544, 3584, 3612, 3632, 3646, 3659, 3672, 3688, 3707,
    3730, 3757, 3789, 3824, 3861, 3901, 3942, 3987, 4040, 4200,
    3505, 3544, 3584, 3612, 3632, 3646, 3659, 3672, 3688, 3707, 3730,
    3757, 3789, 3824, 3861, 3901, 3942, 3987, 4040, 4120
};
static int battery_capacity_down_table_45_600_k4_mei[] __initdata =
{
    3400, 3569, 3587, 3623, 3648, 3663, 3675, 3687, 3701, 3718, 3737,
    3760, 3790, 3822, 3855, 3890, 3927, 3968, 4012, 4057, 4200,
    3569, 3587, 3623, 3648, 3663, 3675, 3687, 3701, 3718, 3737, 3760,
    3790, 3822, 3855, 3890, 3927, 3968, 4012, 4057, 4120

};
// 700mA
static int battery_capacity_down_table_0_700_k4_mei[] __initdata =
{
    3400, 3423, 3442, 3461, 3478, 3495, 3509, 3523, 3535, 3547, 3560,
    3574, 3590, 3608, 3628, 3652, 3680, 3718, 3769, 3856, 4200,
    3423, 3442, 3461, 3478, 3495, 3509, 3523, 3535, 3547, 3560, 3574,
    3590, 3608, 3628, 3652, 3680, 3718, 3769, 3856, 4120
};
static int battery_capacity_down_table_25_700_k4_mei[] __initdata =
{
    3400, 3478, 3520, 3557, 3586, 3607, 3623, 3637, 3650, 3666, 3685,
    3707, 3733, 3763, 3797, 3833, 3872, 3913, 3959, 4012, 4200,
    3478, 3520, 3557, 3586, 3607, 3623, 3637, 3650, 3666, 3685, 3707,
    3733, 3763, 3797, 3833, 3872, 3913, 3959, 4012, 4120
};
static int battery_capacity_down_table_45_700_k4_mei[] __initdata =
{
    3400, 3547, 3568, 3605, 3630, 3645, 3657, 3669, 3684, 3700, 3720,
    3743, 3771, 3803, 3836, 3872, 3910, 3951, 3993, 4033, 4200,
    3547, 3568, 3605, 3630, 3645, 3657, 3669, 3684, 3700, 3720, 3743,
    3771, 3803, 3836, 3872, 3910, 3951, 3993, 4033, 4120
};





/*JHT 1000mAh battery table*/
// 100mA
static int battery_capacity_down_table_0_100_JHT_1000[] __initdata =
{
    /* 0%   5%   10%   15%   20%   25%   30%   35%   40%   45%   50%
     55%   60%   65%   70%   75%   80%   85%   90%   95%  100%    */
    3400, 3520, 3581, 3621, 3655, 3678, 3693, 3703, 3713, 3723, 3738,
    3757, 3778, 3804, 3834, 3865, 3893, 3929, 3995, 4041, 4200,
    3520, 3581, 3621, 3655, 3678, 3693, 3703, 3713, 3723, 3738, 3757,
    3778, 3804, 3834, 3865, 3893, 3929, 3995, 4041, 4120
};
//modified
static int battery_capacity_down_table_25_100_JHT_1000[] __initdata =
{
    3400, 3631, 3657, 3692, 3723, 3739, 3747, 3754, 3764, 3777, 3793, 
    3813, 3842, 3876, 3908, 3942, 3979, 4019, 4064, 4112, 4200,
    3631, 3657, 3692, 3723, 3739, 3747, 3754, 3764, 3777, 3793, 3813,
    3842, 3876, 3908, 3942, 3979, 4019, 4064, 4112, 4120
};
static int battery_capacity_down_table_45_100_JHT_1000[] __initdata =
{
    3400, 3645, 3657, 3692, 3716, 3732, 3744, 3752, 3761, 3775, 3791,
    3816, 3855, 3884, 3914, 3949, 3983, 4029, 4070, 4116, 4200,
    3645, 3657, 3692, 3716, 3732, 3744, 3752, 3761, 3775, 3791, 3816, 
    3855, 3884, 3914, 3949, 3983, 4029, 4070, 4116, 4120
};

// 200mA
static int battery_capacity_down_table_0_200_JHT_1000[] __initdata =
{
    3400, 3481, 3530, 3565, 3593, 3616, 3634, 3649, 3662, 3678, 3695,
    3716, 3737, 3762, 3788, 3816, 3847, 3889, 3951, 4000, 4200,
    3481, 3530, 3565, 3593, 3616, 3634, 3649, 3662, 3678, 3695, 3716,
    3737, 3762, 3788, 3816, 3847, 3889, 3951, 4000, 4120

};
//modified
static int battery_capacity_down_table_25_200_JHT_1000[] __initdata =
{
    3400, 3602, 3636, 3667, 3694, 3709, 3720, 3729, 3741, 3755, 3773,
    3794, 3819, 3849, 3882, 3918, 3957, 3999, 4045, 4094, 4200,
    3602, 3636, 3667, 3694, 3709, 3720, 3729, 3741, 3755, 3773, 3794,
    3819, 3849, 3882, 3918, 3957, 3999, 4045, 4094, 4120
};
static int battery_capacity_down_table_45_200_JHT_1000[] __initdata =
{
    3400, 3623, 3638, 3675, 3698, 3715, 3724, 3733, 3744, 3759, 3776,
    3800, 3835, 3866, 3899, 3933, 3969, 4013, 4055, 4102, 4200,
    3623, 3638, 3675, 3698, 3715, 3724, 3733, 3744, 3759, 3776, 3800,
    3835, 3866, 3899, 3933, 3969, 4013, 4055, 4102, 4120
};

// 300mA
static int battery_capacity_down_table_0_300_JHT_1000[] __initdata =
{
    3400, 3458, 3495, 3525, 3548, 3569, 3587, 3603, 3618, 3635, 3653,
    3674, 3696, 3720, 3745, 3773, 3804, 3846, 3905, 3957, 4200,
    3458, 3495, 3525, 3548, 3569, 3587, 3603, 3618, 3635, 3653, 3674,
    3696, 3720, 3745, 3773, 3804, 3846, 3905, 3957, 4120
};
//modified
static int battery_capacity_down_table_25_300_JHT_1000[] __initdata =
{
    3400, 3576, 3616, 3644, 3667, 3682, 3695, 3706, 3719, 3734, 3754,
    3776, 3801, 3829, 3861, 3896, 3935, 3978, 4024, 4074, 4200,
    3576, 3616, 3644, 3667, 3682, 3695, 3706, 3719, 3734, 3754, 3776,
    3801, 3829, 3861, 3896, 3935, 3978, 4024, 4074, 4120
};
static int battery_capacity_down_table_45_300_JHT_1000[] __initdata =
{
    3400, 3601, 3620, 3657, 3680, 3696, 3705, 3714, 3726, 3741, 3759,
    3783, 3815, 3846, 3878, 3913, 3950, 3994, 4037, 4084, 4200,
    3601, 3620, 3657, 3680, 3696, 3705, 3714, 3726, 3741, 3759, 3783,
    3815, 3846, 3878, 3913, 3950, 3994, 4037, 4084, 4120
};

// 400mA
static int battery_capacity_down_table_0_400_JHT_1000[] __initdata =
{
    3400, 3434, 3460, 3484, 3503, 3522, 3539, 3556, 3573, 3591, 3611,
    3632, 3654, 3678, 3702, 3730, 3761, 3803, 3859, 3914, 4200,
    3434, 3460, 3484, 3503, 3522, 3539, 3556, 3573, 3591, 3611, 3632,
    3654, 3678, 3702, 3730, 3761, 3803, 3859, 3914, 4120
};
//modified
static int battery_capacity_down_table_25_400_JHT_1000[] __initdata =
{
    3400, 3550, 3595, 3621, 3639, 3655, 3669, 3682, 3696, 3713, 3734,
    3757, 3782, 3809, 3840, 3874, 3912, 3956, 4003, 4054, 4200,
    3550, 3595, 3621, 3639, 3655, 3669, 3682, 3696, 3713, 3734, 3757,
    3782, 3809, 3840, 3874, 3912, 3956, 4003, 4054, 4120
};
static int battery_capacity_down_table_45_400_JHT_1000[] __initdata =
{   
    3400, 3579, 3602, 3638, 3662, 3677, 3686, 3695, 3707, 3723, 3742,
    3765, 3794, 3825, 3857, 3893, 3930, 3975, 4018, 4066, 4200,
    3579, 3602, 3638, 3662, 3677, 3686, 3695, 3707, 3723, 3742, 3765,
    3794, 3825, 3857, 3893, 3930, 3975, 4018, 4066, 4120
};

// 500mA
static int battery_capacity_down_table_0_500_JHT_1000[] __initdata =
{
    3400, 3426, 3446, 3466, 3483, 3500, 3515, 3531, 3548, 3565, 3584,
    3604, 3625, 3647, 3671, 3699, 3731, 3771, 3821, 3876, 4200,
    3426, 3446, 3466, 3483, 3500, 3515, 3531, 3548, 3565, 3584, 3604,
    3625, 3647, 3671, 3699, 3731, 3771, 3821, 3876, 4120
};
//modified
static int battery_capacity_down_table_25_500_JHT_1000[] __initdata =
{
    3400, 3528, 3572, 3599, 3618, 3633, 3647, 3661, 3676, 3694, 3715,
    3738, 3764, 3792, 3823, 3857, 3895, 3938, 3985, 4036, 4200,
    3528, 3572, 3599, 3618, 3633, 3647, 3661, 3676, 3694, 3715, 3738,
    3764, 3792, 3823, 3857, 3895, 3938, 3985, 4036, 4120
};
static int battery_capacity_down_table_45_500_JHT_1000[] __initdata =
{
    3400, 3561, 3586, 3621, 3644, 3658, 3668, 3678, 3690, 3707, 3727,
    3750, 3777, 3808, 3840, 3876, 3915, 3960, 4004, 4053, 4200,
    3561, 3586, 3621, 3644, 3658, 3668, 3678, 3690, 3707, 3727, 3750,
    3777, 3808, 3840, 3876, 3915, 3960, 4004, 4053, 4120
};

// 600mA
static int battery_capacity_down_table_0_600_JHT_1000[] __initdata =
{
    3400, 3417, 3432, 3447, 3462, 3477, 3491, 3506, 3522, 3538, 3556,
    3576, 3595, 3615, 3639, 3668, 3700, 3738, 3782, 3837, 4200,
    3417, 3432, 3447, 3462, 3477, 3491, 3506, 3522, 3538, 3556, 3576,
    3595, 3615, 3639, 3668, 3700, 3738, 3782, 3837, 4120
};
//modified
static int battery_capacity_down_table_25_600_JHT_1000[] __initdata =
{
    3400, 3505, 3549, 3576, 3596, 3611, 3625, 3639, 3655, 3674, 3696,
    3719, 3746, 3774, 3805, 3839, 3877, 3919, 3966, 4017, 4200,
    3505, 3549, 3576, 3596, 3611, 3625, 3639, 3655, 3674, 3696, 3719,
    3746, 3774, 3805, 3839, 3877, 3919, 3966, 4017, 4120
};
static int battery_capacity_down_table_45_600_JHT_1000[] __initdata =
{
    3400, 3542, 3569, 3603, 3625, 3639, 3650, 3661, 3673, 3690, 3711,
    3734, 3760, 3790, 3822, 3859, 3899, 3945, 3990, 4040, 4200,
    3542, 3569, 3603, 3625, 3639, 3650, 3661, 3673, 3690, 3711, 3734,
    3760, 3790, 3822, 3859, 3899, 3945, 3990, 4040, 4120

};

// 800mA
static int battery_capacity_down_table_0_800_JHT_1000[] __initdata =
{
    3400, 3414, 3422, 3431, 3439, 3448, 3458, 3467, 3477, 3487, 3498,
    3509, 3519, 3530, 3540, 3550, 3565, 3591, 3640, 3720, 4200,
    3414, 3422, 3431, 3439, 3448, 3458, 3467, 3477, 3487, 3498, 3509,
    3519, 3530, 3540, 3550, 3565, 3591, 3640, 3720, 4120
};
//modified
static int battery_capacity_down_table_25_800_JHT_1000[] __initdata =
{
    3400, 3466, 3504, 3532, 3553, 3573, 3590, 3605, 3621, 3640, 3662,
    3688, 3715, 3745, 3776, 3811, 3848, 3889, 3932, 3983, 4200,
    3466, 3504, 3532, 3553, 3573, 3590, 3605, 3621, 3640, 3662, 3688,
    3715, 3745, 3776, 3811, 3848, 3889, 3932, 3983, 4120
};
static int battery_capacity_down_table_45_800_JHT_1000[] __initdata =
{
    3400, 3498, 3530, 3559, 3579, 3595, 3609, 3622, 3635, 3652, 3673,
    3695, 3721, 3750, 3781, 3816, 3856, 3901, 3947, 3998, 4200,
    3498, 3530, 3559, 3579, 3595, 3609, 3622, 3635, 3652, 3673, 3695,
    3721, 3750, 3781, 3816, 3856, 3901, 3947, 3998, 4120
};


static int __refdata *pTableIndex[3][12]  =
{
 {battery_capacity_down_table_0_100_k4, battery_capacity_down_table_0_200_k4,
  battery_capacity_down_table_0_300_k4, battery_capacity_down_table_0_400_k4,
  battery_capacity_down_table_0_500_k4, battery_capacity_down_table_0_600_k4,
  battery_capacity_down_table_0_700_k4, NULL,
  battery_capacity_up_table_0_ac_800,  battery_capacity_up_table_0_ac_400,
  battery_capacity_up_table_0_usb_k4, battery_capacity_up_table_0_usb_k41},

 {battery_capacity_down_table_25_100_k4, battery_capacity_down_table_25_200_k4,
  battery_capacity_down_table_25_300_k4, battery_capacity_down_table_25_400_k4,
  battery_capacity_down_table_25_500_k4, battery_capacity_down_table_25_600_k4,
  battery_capacity_down_table_25_700_k4, NULL, 
  battery_capacity_up_table_25_ac_800,  battery_capacity_up_table_25_ac_400,
  battery_capacity_up_table_25_usb_k4, battery_capacity_up_table_25_usb_k41},


 {battery_capacity_down_table_45_100_k4, battery_capacity_down_table_45_200_k4,
  battery_capacity_down_table_45_300_k4, battery_capacity_down_table_45_400_k4,
  battery_capacity_down_table_45_500_k4, battery_capacity_down_table_45_600_k4,
  battery_capacity_down_table_45_700_k4, NULL, 
  battery_capacity_up_table_45_ac_800,  battery_capacity_up_table_45_ac_400,
  battery_capacity_up_table_45_usb_k4, battery_capacity_up_table_45_usb_k41}

};


static int __refdata *pTableIndex_k4_mei[3][12]  =
{
 {battery_capacity_down_table_0_100_k4_mei, battery_capacity_down_table_0_200_k4_mei,
  battery_capacity_down_table_0_300_k4_mei, battery_capacity_down_table_0_400_k4_mei,
  battery_capacity_down_table_0_500_k4_mei, battery_capacity_down_table_0_600_k4_mei,
  battery_capacity_down_table_0_700_k4_mei, NULL,
  battery_capacity_up_table_0_ac_800,  battery_capacity_up_table_0_ac_400,
  battery_capacity_up_table_0_usb_k4, battery_capacity_up_table_0_usb_k41},

 {battery_capacity_down_table_25_100_k4_mei, battery_capacity_down_table_25_200_k4_mei,
  battery_capacity_down_table_25_300_k4_mei, battery_capacity_down_table_25_400_k4_mei,
  battery_capacity_down_table_25_500_k4_mei, battery_capacity_down_table_25_600_k4_mei,
  battery_capacity_down_table_25_700_k4_mei, NULL, 
  battery_capacity_up_table_25_ac_800,  battery_capacity_up_table_25_ac_400,
  battery_capacity_up_table_25_usb_k4, battery_capacity_up_table_25_usb_k41},


 {battery_capacity_down_table_45_100_k4_mei, battery_capacity_down_table_45_200_k4_mei,
  battery_capacity_down_table_45_300_k4_mei, battery_capacity_down_table_45_400_k4_mei,
  battery_capacity_down_table_45_500_k4_mei, battery_capacity_down_table_45_600_k4_mei,
  battery_capacity_down_table_45_700_k4_mei, NULL, 
  battery_capacity_up_table_45_ac_800,  battery_capacity_up_table_45_ac_400,
  battery_capacity_up_table_45_usb_k4, battery_capacity_up_table_45_usb_k41}

};

static int __refdata *pTableIndex_JHT1000[3][12]  =
{
 {battery_capacity_down_table_0_100_JHT_1000, battery_capacity_down_table_0_200_JHT_1000,
  battery_capacity_down_table_0_300_JHT_1000, battery_capacity_down_table_0_400_JHT_1000,
  battery_capacity_down_table_0_500_JHT_1000, battery_capacity_down_table_0_600_JHT_1000,
  battery_capacity_down_table_0_800_JHT_1000, NULL,
  battery_capacity_up_table_25_ac_800,  battery_capacity_up_table_25_ac_400,
  battery_capacity_up_table_25_usb_k4, battery_capacity_up_table_25_usb_k41},

 {battery_capacity_down_table_25_100_JHT_1000, battery_capacity_down_table_25_200_JHT_1000,
  battery_capacity_down_table_25_300_JHT_1000, battery_capacity_down_table_25_400_JHT_1000,
  battery_capacity_down_table_25_500_JHT_1000, battery_capacity_down_table_25_600_JHT_1000,
  battery_capacity_down_table_25_800_JHT_1000, NULL, 
  battery_capacity_up_table_25_ac_800,  battery_capacity_up_table_25_ac_400,
  battery_capacity_up_table_25_usb_k4, battery_capacity_up_table_25_usb_k41},


 {battery_capacity_down_table_45_100_JHT_1000, battery_capacity_down_table_45_200_JHT_1000,
  battery_capacity_down_table_45_300_JHT_1000, battery_capacity_down_table_45_400_JHT_1000,
  battery_capacity_down_table_45_500_JHT_1000, battery_capacity_down_table_45_600_JHT_1000,
  battery_capacity_down_table_45_800_JHT_1000, NULL, 
  battery_capacity_up_table_25_ac_800,  battery_capacity_up_table_25_ac_400,
  battery_capacity_up_table_25_usb_k4, battery_capacity_up_table_25_usb_k41}

};

