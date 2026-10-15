#ifndef _FX_KINE_LUNA_H_
#define _FX_KINE_LUNA_H_

#include "FXMath.h"

class CFxKineLUNA
{
public:
	CFxKineLUNA();
	~CFxKineLUNA();

	FX_VOID OnSetCondition6DOF(Vect6 std_Body, Vect6 k_Body, FX_DOUBLE std_L_len, FX_DOUBLE k_L, FX_DOUBLE std_R_len, FX_DOUBLE k_R);
	FX_VOID OnSetLunaBodyDH(Vect4 DH[6], FX_DOUBLE Flange_length, Vect6 lmt_neg, Vect6 lmt_pos);
	
	FX_BOOL OnKine6DOF(Vect6 jv, Matrix4 pgL, Matrix4 pgR);
	FX_BOOL OnInvKine6DOF(Matrix4 pgL, Matrix4 pgR, Vect6 ref_j, Vect6 ret_j);

	FX_VOID OnCalBody6DOF(Vect6 tpos1, Vect6 tpos2, Vect6 ret_pos);
	FX_VOID OnCalBody6DOF_withref(Vect6 refjv, Vect6 tpos1, Vect6 tpos2, Vect6 ret_pos);

private:
	FX_VOID OnKine6DOF_Base(Vect6 jv, Matrix4 pgB, Matrix4 pgL, Matrix4 pgR, Matrix6 jcbL, Matrix6 jcbR);
	FX_BOOL OnCheckJointRange(Vect6 jv);

	FX_BOOL m_kine_init_tag;
	Matrix4 Luna_body[6];
	FX_DOUBLE m_l0;
	FX_DOUBLE m_l1;
	FX_DOUBLE m_l2;
	FX_DOUBLE m_flange;
	FX_DOUBLE m_reach_lmt;

	Vect6 m_body_limit_neg;
	Vect6 m_body_limit_pos;

	Vect6 m_std_body6;
	Vect6 m_k_body6;
	FX_DOUBLE m_std_L_len;
	FX_DOUBLE m_k_L;
	FX_DOUBLE m_std_R_len;
	FX_DOUBLE m_k_R;
	
	FX_DOUBLE m_dArm2Pitch;
	FX_DOUBLE m_dShoulder;

	Matrix4 m_ArmL_Set;
	Matrix4 m_ArmR_Set;

	Matrix4 m_ArmL_Set_Inv;
	Matrix4 m_ArmR_Set_Inv;
};

#endif
