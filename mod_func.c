#include <stdio.h>
#include "hocdec.h"
#define IMPORT extern __declspec(dllimport)
IMPORT int nrnmpi_myid, nrn_nobanner_;

extern void _Gfluct_reg();
extern void _HCaL_reg();
extern void _Ipulse_reg();
extern void _KAHP_reg();
extern void _L_Ca_inact_reg();
extern void _RampIClamp_reg();
extern void _VClamp_reg();
extern void _cagk_reg();
extern void _exp2syn1_reg();
extern void _expsyn1_reg();
extern void _gh_reg();
extern void _kaprox_reg();
extern void _kdrRL_reg();
extern void _leak_reg();
extern void _mAHP_reg();
extern void _na3rp_reg();
extern void _naps_reg();
extern void _netstim1_reg();

void modl_reg(){
	//nrn_mswindll_stdio(stdin, stdout, stderr);
    if (!nrn_nobanner_) if (nrnmpi_myid < 1) {
	fprintf(stderr, "Additional mechanisms from files\n");

fprintf(stderr," Gfluct.mod");
fprintf(stderr," HCaL.mod");
fprintf(stderr," Ipulse.mod");
fprintf(stderr," KAHP.mod");
fprintf(stderr," L_Ca_inact.mod");
fprintf(stderr," RampIClamp.mod");
fprintf(stderr," VClamp.mod");
fprintf(stderr," cagk.mod");
fprintf(stderr," exp2syn1.mod");
fprintf(stderr," expsyn1.mod");
fprintf(stderr," gh.mod");
fprintf(stderr," kaprox.mod");
fprintf(stderr," kdrRL.mod");
fprintf(stderr," leak.mod");
fprintf(stderr," mAHP.mod");
fprintf(stderr," na3rp.mod");
fprintf(stderr," naps.mod");
fprintf(stderr," netstim1.mod");
fprintf(stderr, "\n");
    }
_Gfluct_reg();
_HCaL_reg();
_Ipulse_reg();
_KAHP_reg();
_L_Ca_inact_reg();
_RampIClamp_reg();
_VClamp_reg();
_cagk_reg();
_exp2syn1_reg();
_expsyn1_reg();
_gh_reg();
_kaprox_reg();
_kdrRL_reg();
_leak_reg();
_mAHP_reg();
_na3rp_reg();
_naps_reg();
_netstim1_reg();
}
