/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__KAHP
#define _nrn_initial _nrn_initial__KAHP
#define nrn_cur _nrn_cur__KAHP
#define _nrn_current _nrn_current__KAHP
#define nrn_jacob _nrn_jacob__KAHP
#define nrn_state _nrn_state__KAHP
#define _net_receive _net_receive__KAHP 
#define mcarate mcarate__KAHP 
#define rates rates__KAHP 
#define states states__KAHP 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define gkcamax _p[0]
#define gcamax _p[1]
#define taur _p[2]
#define ik _p[3]
#define ica _p[4]
#define mca _p[5]
#define hca _p[6]
#define q _p[7]
#define cai _p[8]
#define ek _p[9]
#define eca _p[10]
#define qinf _p[11]
#define qtau _p[12]
#define minfca _p[13]
#define mtauca _p[14]
#define hinfca _p[15]
#define htauca _p[16]
#define Dmca _p[17]
#define Dhca _p[18]
#define Dq _p[19]
#define Dcai _p[20]
#define v _p[21]
#define _g _p[22]
#define _ion_ek	*_ppvar[0]._pval
#define _ion_ik	*_ppvar[1]._pval
#define _ion_dikdv	*_ppvar[2]._pval
#define _ion_eca	*_ppvar[3]._pval
#define _ion_ica	*_ppvar[4]._pval
#define _ion_dicadv	*_ppvar[5]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 extern double celsius;
 /* declaration of user functions */
 static void _hoc_mcarate(void);
 static void _hoc_rates(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_KAHP", _hoc_setdata,
 "mcarate_KAHP", _hoc_mcarate,
 "rates_KAHP", _hoc_rates,
 0, 0
};
 /* declare global and static user variables */
#define B B_KAHP
 double B = -17.024;
#define bKCa bKCa_KAHP
 double bKCa = 0.1;
#define cainf cainf_KAHP
 double cainf = 0.0001;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "cainf_KAHP", "mM",
 "gkcamax_KAHP", "S/cm2",
 "gcamax_KAHP", "S/cm2",
 "taur_KAHP", "ms",
 "cai_KAHP", "mM",
 "ik_KAHP", "mA/cm2",
 "ica_KAHP", "mA/cm2",
 0,0
};
 static double cai0 = 0;
 static double delta_t = 0.01;
 static double hca0 = 0;
 static double mca0 = 0;
 static double q0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "cainf_KAHP", &cainf_KAHP,
 "bKCa_KAHP", &bKCa_KAHP,
 "B_KAHP", &B_KAHP,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[6]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"KAHP",
 "gkcamax_KAHP",
 "gcamax_KAHP",
 "taur_KAHP",
 0,
 "ik_KAHP",
 "ica_KAHP",
 0,
 "mca_KAHP",
 "hca_KAHP",
 "q_KAHP",
 "cai_KAHP",
 0,
 0};
 static Symbol* _k_sym;
 static Symbol* _ca_sym;
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 23, _prop);
 	/*initialize range parameters*/
 	gkcamax = 0.05;
 	gcamax = 3e-005;
 	taur = 10;
 	_prop->param = _p;
 	_prop->param_size = 23;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 7, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 prop_ion = need_memb(_k_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[0]._pval = &prop_ion->param[0]; /* ek */
 	_ppvar[1]._pval = &prop_ion->param[3]; /* ik */
 	_ppvar[2]._pval = &prop_ion->param[4]; /* _ion_dikdv */
 prop_ion = need_memb(_ca_sym);
 nrn_promote(prop_ion, 0, 1);
 	_ppvar[3]._pval = &prop_ion->param[0]; /* eca */
 	_ppvar[4]._pval = &prop_ion->param[3]; /* ica */
 	_ppvar[5]._pval = &prop_ion->param[4]; /* _ion_dicadv */
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 static void _update_ion_pointer(Datum*);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _KAHP_reg() {
	int _vectorized = 1;
  _initlists();
 	ion_reg("k", -10000.);
 	ion_reg("ca", -10000.);
 	_k_sym = hoc_lookup("k_ion");
 	_ca_sym = hoc_lookup("ca_ion");
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
     _nrn_thread_reg(_mechtype, 2, _update_ion_pointer);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 23, 7);
  hoc_register_dparam_semantics(_mechtype, 0, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 1, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 2, "k_ion");
  hoc_register_dparam_semantics(_mechtype, 3, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 4, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 5, "ca_ion");
  hoc_register_dparam_semantics(_mechtype, 6, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 KAHP E:/motoneuron pool(4)(LOAD FF)/motoneuron recruitment/KAHP.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 static double FARADAY = 96485.3;
static int _reset;
static char *modelname = "";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int mcarate(_threadargsprotocomma_ double);
static int rates(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[4], _dlist1[4];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   rates ( _threadargscomma_ cai ) ;
   Dcai = B * ica - cai / taur ;
   Dq = ( qinf - q ) / qtau ;
   mcarate ( _threadargscomma_ v ) ;
   Dmca = ( minfca - mca ) / mtauca ;
   Dhca = ( hinfca - hca ) / htauca ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 rates ( _threadargscomma_ cai ) ;
 Dcai = Dcai  / (1. - dt*( ( - ( 1.0 ) / taur ) )) ;
 Dq = Dq  / (1. - dt*( ( ( ( - 1.0 ) ) ) / qtau )) ;
 mcarate ( _threadargscomma_ v ) ;
 Dmca = Dmca  / (1. - dt*( ( ( ( - 1.0 ) ) ) / mtauca )) ;
 Dhca = Dhca  / (1. - dt*( ( ( ( - 1.0 ) ) ) / htauca )) ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   rates ( _threadargscomma_ cai ) ;
    cai = cai + (1. - exp(dt*(( - ( 1.0 ) / taur ))))*(- ( ( B )*( ica ) ) / ( ( - ( 1.0 ) / taur ) ) - cai) ;
    q = q + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / qtau)))*(- ( ( ( qinf ) ) / qtau ) / ( ( ( ( - 1.0 ) ) ) / qtau ) - q) ;
   mcarate ( _threadargscomma_ v ) ;
    mca = mca + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / mtauca)))*(- ( ( ( minfca ) ) / mtauca ) / ( ( ( ( - 1.0 ) ) ) / mtauca ) - mca) ;
    hca = hca + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / htauca)))*(- ( ( ( hinfca ) ) / htauca ) / ( ( ( ( - 1.0 ) ) ) / htauca ) - hca) ;
   }
  return 0;
}
 
static int  rates ( _threadargsprotocomma_ double _lcai ) {
   double _lalphaq , _lbetaq ;
 _lalphaq = _lcai ;
   _lbetaq = bKCa ;
   qtau = 1.0 / ( _lalphaq + _lbetaq ) ;
   qinf = _lalphaq * qtau ;
    return 0; }
 
static void _hoc_rates(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 rates ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int  mcarate ( _threadargsprotocomma_ double _lv ) {
   double _lalphacam , _lbetacam , _lalphacah , _lbetacah ;
 _lalphacam = 0.25 / ( 1.0 + exp ( ( _lv + 20.0 ) / - 5.0 ) ) ;
   _lbetacam = 0.25 / ( 1.0 + exp ( ( _lv + 20.0 ) / 5.0 ) ) ;
   minfca = _lalphacam / ( _lalphacam + _lbetacam ) ;
   mtauca = 1.0 / ( _lalphacam + _lbetacam ) ;
   _lalphacah = 0.025 / ( 1.0 - exp ( ( _lv + 35.0 ) / - 5.0 ) ) ;
   _lbetacah = 0.025 / ( 1.0 - exp ( ( _lv + 35.0 ) / 5.0 ) ) ;
   hinfca = _lalphacah / ( _lalphacah + _lbetacah ) ;
   htauca = 1.0 / ( _lalphacah + _lbetacah ) ;
    return 0; }
 
static void _hoc_mcarate(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 mcarate ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 4;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ek = _ion_ek;
  eca = _ion_eca;
     _ode_spec1 (_p, _ppvar, _thread, _nt);
   }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 4; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
  ek = _ion_ek;
  eca = _ion_eca;
 _ode_matsol_instance1(_threadargs_);
 }}
 extern void nrn_update_ion_pointer(Symbol*, Datum*, int, int);
 static void _update_ion_pointer(Datum* _ppvar) {
   nrn_update_ion_pointer(_k_sym, _ppvar, 0, 0);
   nrn_update_ion_pointer(_k_sym, _ppvar, 1, 3);
   nrn_update_ion_pointer(_k_sym, _ppvar, 2, 4);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 3, 0);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 4, 3);
   nrn_update_ion_pointer(_ca_sym, _ppvar, 5, 4);
 }

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  cai = cai0;
  hca = hca0;
  mca = mca0;
  q = q0;
 {
   cai = cainf ;
   rates ( _threadargscomma_ cai ) ;
   mcarate ( _threadargscomma_ v ) ;
   q = qinf ;
   mca = minfca ;
   hca = hinfca ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
  ek = _ion_ek;
  eca = _ion_eca;
 initmodel(_p, _ppvar, _thread, _nt);
  }
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   ica = gcamax * pow( mca , 2.0 ) * hca * ( v - eca ) ;
   ik = gkcamax * q * ( v - ek ) ;
   }
 _current += ik;
 _current += ica;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
  ek = _ion_ek;
  eca = _ion_eca;
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ double _dica;
 double _dik;
  _dik = ik;
  _dica = ica;
 _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
  _ion_dikdv += (_dik - ik)/.001 ;
  _ion_dicadv += (_dica - ica)/.001 ;
 	}
 _g = (_g - _rhs)/.001;
  _ion_ik += ik ;
  _ion_ica += ica ;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
  ek = _ion_ek;
  eca = _ion_eca;
 {   states(_p, _ppvar, _thread, _nt);
  }  }}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(cai) - _p;  _dlist1[0] = &(Dcai) - _p;
 _slist1[1] = &(q) - _p;  _dlist1[1] = &(Dq) - _p;
 _slist1[2] = &(mca) - _p;  _dlist1[2] = &(Dmca) - _p;
 _slist1[3] = &(hca) - _p;  _dlist1[3] = &(Dhca) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "KAHP.mod";
static const char* nmodl_file_text = 
  "\n"
  " COMMENT\n"
  " \n"
  " KAHP.mod\n"
  " \n"
  " Calcium-dependent potassium channel responsible for mAHP in motoneurons\n"
  " Simplified calcium channel that provides Ca for the KCa conductance is included.\n"
  " Originally taken from Powers et al, 2012.\n"
  " 	\n"
  " ENDCOMMENT\n"
  "\n"
  " NEURON {\n"
  " 	SUFFIX KAHP\n"
  " 	USEION k READ ek WRITE ik\n"
  " 	USEION ca READ eca WRITE ica\n"
  " 	RANGE  gkcamax,gcamax,ik,cai,ica,taur\n"
  " 	GLOBAL bKCa \n"
  " }\n"
  "\n"
  " \n"
  " UNITS {\n"
  " 	(mA) = (milliamp)\n"
  " 	(mV) = (millivolt)\n"
  " 	(S) = (siemens)\n"
  " 	(um) = (micron)\n"
  " 	(molar) = (1/liter)			: moles do not appear in units\n"
  " 	(mM)	= (millimolar)\n"
  " 	(msM)	= (ms mM)\n"
  " 	FARADAY = (faraday) (coulomb)\n"
  " } \n"
  " \n"
  " PARAMETER {\n"
  " 	gkcamax = 0.05   	(S/cm2)	\n"
  "	gcamax = 3e-5		(S/cm2)\n"
  "   \n"
  "  cainf=0.0001		(mM)\n"
  " 	taur	= 10		(ms)		: rate of calcium removal\n"
  " \n"
  " 	bKCa   = 0.1	  : max deact rate \n"
  "  B=-17.024       :scaling constant\n"
  " 	celsius		(degC)\n"
  " } \n"
  " \n"
  " \n"
  " ASSIGNED {\n"
  " 	ik 		(mA/cm2)\n"
  " 	v 		(mV)\n"
  "	ica 	(mA/cm2)\n"
  " 	ek		(mV)\n"
  "	eca		(mV)\n"
  " 	qinf\n"
  " 	qtau 		(ms)\n"
  "  minfca\n"
  "  mtauca  (ms)\n"
  "  hinfca\n"
  "  htauca  (ms) \n"
  " }\n"
  "  \n"
  " \n"
  " STATE {\n"
  " mca\n"
  " hca \n"
  " q \n"
  " cai (mM)\n"
  "}\n"
  " \n"
  " INITIAL { \n"
  "	cai=cainf\n"
  " 	rates(cai)\n"
  "	mcarate(v)\n"
  " 	q = qinf\n"
  "	mca=minfca\n"
  "  hca=hinfca\n"
  " }\n"
  " \n"
  " BREAKPOINT {\n"
  "         SOLVE states METHOD cnexp\n"
  "	ica = gcamax*mca^2*hca*(v - eca)   :CaN\n"
  " 	ik =  gkcamax *q* (v - ek)\n"
  " } \n"
  " \n"
  "\n"
  "DERIVATIVE states { \n"
  "	 \n"
  "         rates(cai) \n"
  " 	       cai' = B*ica-cai/taur  \n"
  "         q' = (qinf-q)/qtau\n"
  "         mcarate(v)    \n"
  "         mca' = (minfca - mca)/mtauca\n"
  "         hca' = (hinfca - hca)/htauca\n"
  "}\n"
  "PROCEDURE rates(cai(mM)) {  LOCAL alphaq,betaq\n"
  "					 \n"
  "         alphaq=cai           :related to [Ca]in\n"
  "         betaq = bKCa\n"
  "         qtau = 1/(alphaq+betaq)\n"
  "         qinf = alphaq*qtau\n"
  "				 \n"
  " }\n"
  "\n"
  "PROCEDURE mcarate(v (mV)) {\n"
  "   LOCAL alphacam,betacam ,  alphacah,betacah\n"
  "   alphacam=0.25/(1+exp((v+20)/-5))\n"
  "   betacam=0.25/(1+exp((v+20)/5))\n"
  "   minfca = alphacam/(alphacam+betacam) \n"
  "   mtauca= 1/(alphacam+betacam)\n"
  "   \n"
  "   alphacah=0.025/(1-exp((v+35)/-5))\n"
  "   betacah =0.025/(1-exp((v+35)/5))\n"
  "   \n"
  "   hinfca= alphacah/(alphacah+betacah)\n"
  "   htauca= 1/(alphacah+betacah)\n"
  "    \n"
  "}\n"
  ;
#endif
