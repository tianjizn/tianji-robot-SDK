#include "FXLunaBodyKinematics.h"
#include "FXMatrix.h"
#include "FXLog.h"

CFxKineLUNA::CFxKineLUNA()
{
	FX_INT32 i = 0;
	for (i = 0; i < 6; i++)
	{
		FX_IdentM44(Luna_body[i]);
		m_body_limit_neg[i] = 0.0;
		m_body_limit_pos[i] = 0.0;
	}

	FX_IdentM44(m_ArmL_Set);
	FX_IdentM44(m_ArmR_Set);

	m_l0 = 0.0;
	m_l1 = 0.0;
	m_l2 = 0.0;
	m_flange = 0.0;
	m_reach_lmt = 0.0;

	m_kine_init_tag = FX_FALSE;
}

CFxKineLUNA::~CFxKineLUNA()
{
}

FX_VOID CFxKineLUNA::OnSetCondition6DOF(Vect6 std_Body, Vect6 k_Body, FX_DOUBLE std_L_len, FX_DOUBLE k_L, FX_DOUBLE std_R_len, FX_DOUBLE k_R)
{
	for (FX_INT32 i = 0; i < 6; i++)
	{
		m_std_body6[i] = std_Body[i];
		m_k_body6[i] = k_Body[i];
	}
	m_std_L_len = std_L_len;
	m_k_L = k_L;
	m_std_R_len = std_R_len;
	m_k_R = k_R;
}

// alpha-a-d-theta
FX_VOID CFxKineLUNA::OnSetLunaBodyDH(Vect4 DH[6], FX_DOUBLE Flange_length, Vect6 lmt_neg, Vect6 lmt_pos)
{
	m_dArm2Pitch = 214.6;
	m_dShoulder = 100.0;

	FX_INT32 i = 0;
	for (i = 0; i < 6; i++)
	{
		FX_IdentM44(Luna_body[i]);
		m_body_limit_neg[i] = lmt_neg[i];
		m_body_limit_pos[i] = lmt_pos[i];
	}

	m_l0 = DH[1][1];
	m_l1 = DH[2][1];
	m_l2 = DH[3][1];
	m_flange = Flange_length;
	m_reach_lmt = m_l0 + m_l1 + m_l2;

	Luna_body[0][0][0] = 0;
	Luna_body[0][0][1] = 0;
	Luna_body[0][0][2] = 1;
	Luna_body[0][0][3] = 0;
	Luna_body[0][1][0] = 0;
	Luna_body[0][1][1] = -1;
	Luna_body[0][1][2] = 0;
	Luna_body[0][1][3] = 0;
	Luna_body[0][2][0] = 1;
	Luna_body[0][2][1] = 0;
	Luna_body[0][2][2] = 0;
	Luna_body[0][2][3] = 0;

	Luna_body[1][0][0] = 0;
	Luna_body[1][0][1] = 1;
	Luna_body[1][0][2] = 0;
	Luna_body[1][0][3] = m_l0;
	Luna_body[1][1][0] = 0;
	Luna_body[1][1][1] = 0;
	Luna_body[1][1][2] = 1;
	Luna_body[1][2][0] = 1;
	Luna_body[1][2][1] = 0;
	Luna_body[1][2][2] = 0;

	Luna_body[2][1][3] = m_l1;

	Luna_body[3][1][3] = m_l2;

	Luna_body[4][0][0] = 0;
	Luna_body[4][0][1] = 0;
	Luna_body[4][0][2] = 1;
	Luna_body[4][1][0] = 1;
	Luna_body[4][1][1] = 0;
	Luna_body[4][1][2] = 0;
	Luna_body[4][2][0] = 0;
	Luna_body[4][2][1] = 1;
	Luna_body[4][2][2] = 0;

	Luna_body[5][0][0] = 0;
	Luna_body[5][0][1] = 0;
	Luna_body[5][0][2] = 1;
	Luna_body[5][0][3] = 0;
	Luna_body[5][1][0] = 0;
	Luna_body[5][1][1] = -1;
	Luna_body[5][1][2] = 0;
	Luna_body[5][1][3] = 0;
	Luna_body[5][2][0] = 1;
	Luna_body[5][2][1] = 0;
	Luna_body[5][2][2] = 0;
	Luna_body[5][2][3] = 0;

	FX_IdentM44(m_ArmL_Set);
	FX_IdentM44(m_ArmR_Set);

	m_ArmL_Set[1][1] = 0;
	m_ArmL_Set[1][2] = 1;
	m_ArmL_Set[1][3] = m_dShoulder;
	m_ArmL_Set[2][1] = 1;
	m_ArmL_Set[2][2] = 0; // m_ArmL_Set[2][3] = m_dArm2Pitch;

	m_ArmR_Set[1][1] = 0;
	m_ArmR_Set[1][2] = -1;
	m_ArmR_Set[1][3] = -m_dShoulder;
	m_ArmR_Set[2][1] = -1;
	m_ArmR_Set[2][2] = 0; // m_ArmR_Set[2][3] = m_dArm2Pitch;

	MatrixInv44(m_ArmL_Set, m_ArmL_Set_Inv);
	MatrixInv44(m_ArmR_Set, m_ArmR_Set_Inv);

	m_kine_init_tag = FX_TRUE;
}

FX_BOOL CFxKineLUNA::OnKine6DOF(Vect6 jv, Matrix4 pgL, Matrix4 pgR)
{
	if (!m_kine_init_tag)
	{
		FX_LOG_ERRO("Please set up luna's body config first.");
		return FX_FALSE;
	}

	Matrix4 T[6];
	Matrix4 T_total;
	Matrix4 Tmp;
	FX_IdentM44(T_total);
	FX_IdentM44(Tmp);

	for (FX_INT32 i = 0; i < 6; i++)
	{
		Matrix4 tmp_matics_ = {{0}};
		FX_RotEuler_XYZ(0, 0, jv[i], tmp_matics_);
		FX_MMM44(Luna_body[i], tmp_matics_, T[i]);
		FX_MMM44(Tmp, T[i], T_total);
		FX_M44Copy(T_total, Tmp);
	}

	T_total[0][3] += T_total[0][2] * m_flange;
	T_total[1][3] += T_total[1][2] * m_flange;
	T_total[2][3] += T_total[2][2] * m_flange;

	FX_MMM44(T_total, m_ArmL_Set, pgL);
	FX_MMM44(T_total, m_ArmR_Set, pgR);

	return FX_TRUE;
}

FX_BOOL OnSolveEulerAngle(Matrix4 m, Vect3 ret)
{
	FX_DOUBLE cb;
	ret[1] = FX_ATan2(m[2][2], FX_Sqrt(m[0][2] * m[0][2] + m[1][2] * m[1][2])) * FXARM_R2D;
	cb = FX_COS_DEG(ret[1]);
	if (FX_Fabs(cb) > 0.05 * FXARM_D2R)
	{
		ret[0] = FX_ATan2(-m[0][2] / cb, m[1][2] / cb) * FXARM_R2D;
		ret[2] = FX_ATan2(-m[2][0] / cb, -m[2][1] / cb) * FXARM_R2D;
		return FX_TRUE;
	}
	ret[0] = 0;
	ret[2] = 0;
	return FX_FALSE;
}

FX_BOOL CFxKineLUNA::OnCheckJointRange(Vect6 jv)
{
	FX_BOOL over_limit_tag_ = FX_FALSE;
	for (FX_INT32 i = 0; i < 6; i++)
	{
		if (jv[i] < m_body_limit_neg[i] || jv[i] > m_body_limit_pos[i])
		{
			over_limit_tag_ = FX_TRUE;
		}
	}

	return over_limit_tag_;
}

FX_DOUBLE CheckPoseValidity(Matrix4 left, Matrix4 right)
{
	FX_DOUBLE err = 0.0;
	for (FX_INT32 i = 0; i < 4; i++)
	{
		for (FX_INT32 j = 0; j < 4; j++)
		{
			err += FX_Fabs(left[i][j] - right[i][j]);
		}
	}
	return err;
}

FX_BOOL CFxKineLUNA::OnInvKine6DOF(Matrix4 pgL, Matrix4 pgR, Vect6 ref_j, Vect6 ret_j)
{
	if (!m_kine_init_tag)
	{
		FX_LOG_ERRO("Please set up luna's body config first.");
		return FX_FALSE;
	}

	Matrix4 Leg_EE = {{0}};
	Matrix4 Leg_EE_init = {{0}};

	FX_MMM44(pgL, m_ArmL_Set_Inv, Leg_EE_init);
	// Check the validity of shoulders'pose.
	Matrix4 test_right = {{0}};
	FX_MMM44(pgR, m_ArmR_Set_Inv, test_right);
	FX_DOUBLE err = CheckPoseValidity(Leg_EE_init, test_right);
	FX_LOG_DEBG("err = %f", err);
	if (err > 0.1)
	{
		FX_LOG_ERRO("Incorrect pose; please check the input.");
		return FX_FALSE;
	}

	Leg_EE_init[0][3] -= Leg_EE_init[0][2] * m_flange;
	Leg_EE_init[1][3] -= Leg_EE_init[1][2] * m_flange;
	Leg_EE_init[2][3] -= Leg_EE_init[2][2] * m_flange;

	// Reachable Space Check
	FX_DOUBLE range_check = FX_Sqrt(Leg_EE_init[0][3] * Leg_EE_init[0][3] + Leg_EE_init[1][3] * Leg_EE_init[1][3] + Leg_EE_init[2][3] * Leg_EE_init[2][3]);
	if (range_check > m_reach_lmt)
	{
		FX_LOG_ERRO("Over Range Limit\n");
		return FX_FALSE;
	}

	// J1
	FX_DOUBLE j1 = FX_ATan2(-Leg_EE_init[1][3], Leg_EE_init[2][3]);
	ret_j[0] = j1 * FXARM_R2D;

	{
		// T01
		Matrix4 T[3] = {{{0}}};
		Matrix4 tmp_matics_ = {{0}};

		FX_RotEuler_XYZ(0.0, 0.0, ret_j[0], tmp_matics_);
		FX_MMM44(Luna_body[0], tmp_matics_, T[0]);
		FX_MMM44(T[0], Luna_body[1], T[1]);
		MatrixInv44(T[1], T[2]);
		FX_MMM44(T[2], Leg_EE_init, Leg_EE);
	}

	// J3
	FX_DOUBLE r = FX_Sqrt(Leg_EE[1][3] * Leg_EE[1][3] + Leg_EE[2][3] * Leg_EE[2][3]);
	FX_DOUBLE j1_range_check = FX_Sqrt(Leg_EE[0][3] * Leg_EE[0][3] + Leg_EE[1][3] * Leg_EE[1][3] + Leg_EE[2][3] * Leg_EE[2][3]);
	FX_DOUBLE cos_j3 = (j1_range_check * j1_range_check - 2 * 300.0 * 300.0) / 180000.0;
	if (FX_Fabs(cos_j3) > 1.0)
	{
		cos_j3 = FX_Fabs(cos_j3) / cos_j3;
	}
	FX_DOUBLE j3[2] = {0};
	j3[0] = FX_ACOS(cos_j3) * FXARM_R2D;
	j3[1] = -FX_ACOS(cos_j3) * FXARM_R2D;

	// J2
	FX_DOUBLE j2[2] = {0};
	j2[0] = (FX_ATan2(r, Leg_EE[0][3]) - FX_ATan2(300.0 * FX_SIN_DEG(j3[1]), 300.0 + 300.0 * FX_COS_DEG(j3[1]))) * FXARM_R2D - 90;
	j2[1] = (FX_ATan2(r, Leg_EE[0][3]) - FX_ATan2(300.0 * FX_SIN_DEG(j3[0]), 300.0 + 300.0 * FX_COS_DEG(j3[0]))) * FXARM_R2D - 90;

	FX_DOUBLE diff1 = FX_Fabs(j3[1] - ref_j[2]) + FX_Fabs(j2[0] - ref_j[1]);
	FX_DOUBLE diff2 = FX_Fabs(j3[0] - ref_j[2]) + FX_Fabs(j2[1] - ref_j[1]);

	if (diff1 < diff2)
	{
		ret_j[1] = j2[0];
		ret_j[2] = j3[1];
	}
	else
	{
		ret_j[1] = j2[1];
		ret_j[2] = j3[0];
	}

	// T03
	Matrix4 T[3];
	Matrix4 T03 = {{0}};
	Matrix4 Tmp = {{0}};
	FX_IdentM44(Tmp);
	for (FX_INT32 i = 0; i < 3; i++)
	{
		Matrix4 tmp_matics_ = {{0}};
		FX_RotEuler_XYZ(0, 0, ret_j[i], tmp_matics_);
		FX_MMM44(Luna_body[i], tmp_matics_, T[i]);
		FX_MMM44(Tmp, T[i], T03);
		FX_M44Copy(T03, Tmp);
	}

	Matrix4 T03_inv = {{0}};
	MatrixInv44(T03, T03_inv);

	// J4-J6
	Matrix4 T36 = {{0}};
	FX_MMM44(T03_inv, Leg_EE_init, T36);
	OnSolveEulerAngle(T36, &ret_j[3]);

	FX_LOG_DEBG("%f %f %f %f %f %f",ret_j[0],ret_j[1],ret_j[2],ret_j[3],ret_j[4],ret_j[5]);

	// Joint Range Check
	if (OnCheckJointRange(ret_j) == FX_TRUE)
	{
		FX_LOG_ERRO("Joints exceed limit.");
		return FX_FALSE;
	}
	else
	{
		return FX_TRUE;
	}
}

FX_VOID CFxKineLUNA::OnKine6DOF_Base(Vect6 jv, Matrix4 pgB, Matrix4 pgL, Matrix4 pgR, Matrix6 jcbL, Matrix6 jcbR)
{
	Matrix4 T[6];
	Matrix4 T_total;
	Matrix4 Tmp;
	FX_IdentM44(T_total);
	FX_IdentM44(Tmp);

	for (FX_INT32 i = 0; i < 6; i++)
	{
		Matrix4 tmp_matics_ = {{0}};
		FX_RotEuler_XYZ(0, 0, jv[i], tmp_matics_);
		FX_MMM44(Luna_body[i], tmp_matics_, T[i]);
		FX_MMM44(Tmp, T[i], T_total);
		FX_M44Copy(T_total, Tmp);
	}

	T_total[0][3] += T_total[0][2] * m_flange;
	T_total[1][3] += T_total[1][2] * m_flange;
	T_total[2][3] += T_total[2][2] * m_flange;

	FX_M44Copy(T_total, pgB);

	Matrix4 T_L;
	FX_MMM44(T_total, m_ArmL_Set, T_L);
	FX_M44Copy(T_L, pgL);

	Matrix4 T_R;
	FX_MMM44(T_total, m_ArmR_Set, T_R);
	FX_M44Copy(T_R, pgR);

	Matrix4 T_prev[6];
	FX_IdentM44(T_prev[0]);
	for (FX_INT32 i = 1; i < 6; i++)
	{
		FX_MMM44(T_prev[i - 1], T[i - 1], T_prev[i]);
	}

	for (FX_INT32 i = 0; i < 6; i++)
	{
		Vect3 z_axis;
		z_axis[0] = T_prev[i][0][2];
		z_axis[1] = T_prev[i][1][2];
		z_axis[2] = T_prev[i][2][2];

		Vect3 p_prev;
		p_prev[0] = T_prev[i][0][3];
		p_prev[1] = T_prev[i][1][3];
		p_prev[2] = T_prev[i][2][3];

		Vect3 p_L;
		p_L[0] = T_L[0][3];
		p_L[1] = T_L[1][3];
		p_L[2] = T_L[2][3];

		Vect3 p_R;
		p_R[0] = T_R[0][3];
		p_R[1] = T_R[1][3];
		p_R[2] = T_R[2][3];

		Vect3 dp_L, dp_R;
		dp_L[0] = p_L[0] - p_prev[0];
		dp_L[1] = p_L[1] - p_prev[1];
		dp_L[2] = p_L[2] - p_prev[2];

		dp_R[0] = p_R[0] - p_prev[0];
		dp_R[1] = p_R[1] - p_prev[1];
		dp_R[2] = p_R[2] - p_prev[2];

		Vect3 cross_L, cross_R;
		FX_VectCross(z_axis, dp_L, cross_L);
		FX_VectCross(z_axis, dp_R, cross_R);

		jcbL[0][i] = cross_L[0] * 0.001;
		jcbL[1][i] = cross_L[1] * 0.001;
		jcbL[2][i] = cross_L[2] * 0.001;
		jcbL[3][i] = z_axis[0];
		jcbL[4][i] = z_axis[1];
		jcbL[5][i] = z_axis[2];

		jcbR[0][i] = cross_L[0] * 0.001;
		jcbR[1][i] = cross_L[1] * 0.001;
		jcbR[2][i] = cross_L[2] * 0.001;
		jcbR[3][i] = z_axis[0];
		jcbR[4][i] = z_axis[1];
		jcbR[5][i] = z_axis[2];
	}
}

static FX_DOUBLE CalLen6DOF(Matrix4 ps, Vect3 t, Vect3 ret_vct)
{
	FX_DOUBLE a, b, c;
	a = t[0] - ps[0][3];
	b = t[1] - ps[1][3];
	c = t[2] - ps[2][3];
	ret_vct[0] = a;
	ret_vct[1] = b;
	ret_vct[2] = c;
	return FX_VectNorm_RetLen(ret_vct);
}

static FX_DOUBLE CalForce6DOF(FX_DOUBLE k, FX_DOUBLE stdlen, Matrix4 ps, Vect3 pos, Vect3 ret_vct)
{
	FX_DOUBLE td = (CalLen6DOF(ps, pos, ret_vct) - stdlen) * k;
	ret_vct[0] *= td;
	ret_vct[1] *= td;
	ret_vct[2] *= td;
	return td;
}

static void CalFT6DOF(FX_DOUBLE k, FX_DOUBLE stdlen, Matrix4 pb, Matrix4 ps, Vect3 pos, Vect6 ret_tf)
{
	Vect3 ret_vct;
	CalForce6DOF(k, stdlen, ps, pos, ret_vct);

	Vect3 vct;
	vct[0] = (ps[0][3] - pb[0][3]) * 0.001;
	vct[1] = (ps[1][3] - pb[1][3]) * 0.001;
	vct[2] = (ps[2][3] - pb[2][3]) * 0.001;

	Vect3 tq;
	FX_VectCross(vct, ret_vct, tq);

	ret_tf[0] = ret_vct[0];
	ret_tf[1] = ret_vct[1];
	ret_tf[2] = ret_vct[2];
	ret_tf[3] = tq[0];
	ret_tf[4] = tq[1];
	ret_tf[5] = tq[2];
}

FX_VOID CFxKineLUNA::OnCalBody6DOF(Vect6 tpos1, Vect6 tpos2, Vect6 ret_pos)
{
	FX_DOUBLE rfact = 0.1;
	FX_DOUBLE len1 = m_std_L_len;
	FX_DOUBLE len2 = m_std_R_len;

	FX_DOUBLE k1 = m_k_L;
	FX_DOUBLE k2 = m_k_R;

	FX_DOUBLE Jzpos2[6];
	for (FX_INT32 i = 0; i < 6; i++)
	{
		Jzpos2[i] = m_std_body6[i];
	}

	Matrix4 pg1, pg2, bm;
	Matrix6 jcb1, jcb2;

	OnKine6DOF_Base(Jzpos2, bm, pg1, pg2, jcb1, jcb2);

	Vect3 t1, t2;
	for (FX_INT32 i = 0; i < 3; i++)
	{
		t1[i] = tpos1[i];
		t2[i] = tpos2[i];
	}

	Vect6 FN1, FN2;
	CalFT6DOF(k1, len1, bm, pg1, t1, FN1);
	CalFT6DOF(k2, len2, bm, pg2, t2, FN2);

	Vect6 D;
	FX_DOUBLE max_err = 10;

	for (FX_INT32 i = 0; i < 6; i++)
	{
		D[i] = (FN1[i] + FN2[i]) / m_k_body6[i];
		Jzpos2[i] += D[i] * rfact;
	}

	for (FX_INT32 i = 0; i < 6; i++)
	{
		FX_DOUBLE err = FX_Fabs(D[i]);
		if (err > max_err)
			max_err = err;
	}

	int count = 0;
	while (max_err > 0.0001 && count < 1000)
	{
		FX_LOG_DEBG("[%f %f %f %f %f %f] ", Jzpos2[0], Jzpos2[1], Jzpos2[2], Jzpos2[3], Jzpos2[4], Jzpos2[5]);
		OnKine6DOF_Base(Jzpos2, bm, pg1, pg2, jcb1, jcb2);
		count++;
		FX_LOG_DEBG("<%d>%f\n", count, max_err);
		CalFT6DOF(k1, len1, bm, pg1, t1, FN1);
		CalFT6DOF(k2, len2, bm, pg2, t2, FN2);

		Vect6 FNB;
		for (FX_INT32 i = 0; i < 6; i++)
		{
			FNB[i] = (m_std_body6[i] - Jzpos2[i]) * m_k_body6[i];
			D[i] = (FN1[i] + FN2[i] + FNB[i]) / m_k_body6[i];
			Jzpos2[i] += D[i] * rfact;
		}

		max_err = 0;
		for (FX_INT32 i = 0; i < 6; i++)
		{
			FX_DOUBLE err = FX_Fabs(D[i]);
			if (err > max_err)
				max_err = err;
		}
	}

	for (FX_INT32 i = 0; i < 6; i++)
	{
		ret_pos[i] = Jzpos2[i];
	}
}

FX_VOID CFxKineLUNA::OnCalBody6DOF_withref(Vect6 refjv, Vect6 tpos1, Vect6 tpos2, Vect6 ret_pos)
{
	FX_DOUBLE rfact = 0.1;
	FX_DOUBLE len1 = m_std_L_len;
	FX_DOUBLE len2 = m_std_R_len;

	FX_DOUBLE k1 = m_k_L;
	FX_DOUBLE k2 = m_k_R;

	FX_DOUBLE Jzpos2[6];
	for (FX_INT32 i = 0; i < 6; i++)
	{
		Jzpos2[i] = refjv[i];
	}

	Matrix4 pg1, pg2, bm;
	Matrix6 jcb1, jcb2;

	OnKine6DOF_Base(Jzpos2, bm, pg1, pg2, jcb1, jcb2);

	Vect3 t1, t2;
	for (FX_INT32 i = 0; i < 3; i++)
	{
		t1[i] = tpos1[i];
		t2[i] = tpos2[i];
	}

	Vect6 FN1, FN2;
	CalFT6DOF(k1, len1, bm, pg1, t1, FN1);
	CalFT6DOF(k2, len2, bm, pg2, t2, FN2);

	Vect6 D;
	for (FX_INT32 i = 0; i < 6; i++)
	{
		D[i] = (FN1[i] + FN2[i]) / m_k_body6[i];
		Jzpos2[i] += D[i] * rfact;
	}

	FX_DOUBLE max_err = 0;
	for (FX_INT32 i = 0; i < 6; i++)
	{
		FX_DOUBLE err = FX_Fabs(D[i]);
		if (err > max_err)
			max_err = err;
	}

	while (max_err > 0.0001)
	{
		OnKine6DOF_Base(Jzpos2, bm, pg1, pg2, jcb1, jcb2);

		CalFT6DOF(k1, len1, bm, pg1, t1, FN1);
		CalFT6DOF(k2, len2, bm, pg2, t2, FN2);

		Vect6 FNB;
		for (FX_INT32 i = 0; i < 6; i++)
		{
			FNB[i] = (m_std_body6[i] - Jzpos2[i]) * m_k_body6[i];
			D[i] = (FN1[i] + FN2[i] + FNB[i]) / m_k_body6[i];
			Jzpos2[i] += D[i] * rfact;
		}

		max_err = 0;
		for (FX_INT32 i = 0; i < 6; i++)
		{
			FX_DOUBLE err = FX_Fabs(D[i]);
			if (err > max_err)
				max_err = err;
		}
	}

	for (FX_INT32 i = 0; i < 6; i++)
	{
		ret_pos[i] = Jzpos2[i];
	}
}
