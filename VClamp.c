/* Created by Language version: 7.7.0 */
/* NOT VECTORIZED */
#define NRN_VECTORIZED 0
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
 
#define nrn_init _nrn_init__TVClamp
#define _nrn_initial _nrn_initial__TVClamp
#define nrn_cur _nrn_cur__TVClamp
#define _nrn_current _nrn_current__TVClamp
#define nrn_jacob _nrn_jacob__TVClamp
#define nrn_state _nrn_state__TVClamp
#define _net_receive _net_receive__TVClamp 
#define vstim vstim__TVClamp 
 
#define _threadargscomma_ /**/
#define _threadargsprotocomma_ /**/
#define _threadargs_ /**/
#define _threadargsproto_ /**/
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 static double *_p; static Datum *_ppvar;
 
#define t nrn_threads->_t
#define dt nrn_threads->_dt
#define rs _p[0]
#define i _p[1]
#define vc _p[2]
#define dur (_p + 3)
#define amp (_p + 6)
#define ic _p[9]
#define tc2 _p[10]
#define tc3 _p[11]
#define on _p[12]
#define _g _p[13]
#define _nd_area  *_ppvar[0]._pval
 
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
 /* external NEURON variables */
 /* declaration of user functions */
 static double _hoc_vstim();
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

 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _p = _prop->param; _ppvar = _prop->dparam;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 "vstim", _hoc_vstim,
 0, 0
};
 /* declare global and static user variables */
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "rs", "megohm",
 "i", "nA",
 "vc", "mV",
 "dur", "ms",
 "amp", "mV",
 0,0
};
 static double delta_t = 1;
 static double v = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
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
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"TVClamp",
 "rs",
 0,
 "i",
 "vc",
 "dur[3]",
 "amp[3]",
 0,
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 14, _prop);
 	/*initialize range parameters*/
 	rs = 3;
  }
 	_prop->param = _p;
 	_prop->param_size = 14;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 2, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _VClamp_reg() {
	int _vectorized = 0;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 0,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 14, 2);
  hoc_register_dparam_semantics(_mechtype, 0, "area");
  hoc_register_dparam_semantics(_mechtype, 1, "pntproc");
 	hoc_register_cvode(_mechtype, _ode_count, 0, 0, 0);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 TVClamp E:/motoneuron pool(4)(LOAD FF)/motoneuron recruitment/VClamp.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "svclmp.mod";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int vstim();
 
static int  vstim (  ) {
   on = 1.0 ;
   if ( t < dur [ 0 ] ) {
     vc = amp [ 0 ] ;
     }
   else if ( t < tc2 ) {
     vc = ( ( amp [ 1 ] - amp [ 0 ] ) / dur [ 1 ] ) * ( t - dur [ 0 ] ) + amp [ 0 ] ;
     }
   else if ( t < tc3 ) {
     vc = ( ( amp [ 2 ] - amp [ 1 ] ) / dur [ 2 ] ) * ( t - tc2 ) + amp [ 1 ] ;
     }
   else {
     vc = amp [ 2 ] ;
     on = 0.0 ;
     }
   if ( on ) {
     }
   else {
     ic = 0.0 ;
     }
   
/*VERBATIM*/
        return 0;
  return 0; }
 
static double _hoc_vstim(void* _vptr) {
 double _r;
    _hoc_setdata(_vptr);
 _r = 1.;
 vstim (  );
 return(_r);
}
 
static int _ode_count(int _type){ hoc_execerror("TVClamp", "cannot be used with CVODE"); return 0;}

static void initmodel() {
  int _i; double _save;_ninits++;
 _save = t;
 t = 0.0;
{
 {
   tc2 = dur [ 0 ] + dur [ 1 ] ;
   tc3 = tc2 + dur [ 2 ] ;
   on = 0.0 ;
   }
  _sav_indep = t; t = _save;

}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if EXTRACELLULAR
 _nd = _ml->_nodelist[_iml];
 if (_nd->_extnode) {
    _v = NODEV(_nd) +_nd->_extnode->_v[0];
 }else
#endif
 {
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 }
 v = _v;
 initmodel();
}}

static double _nrn_current(double _v){double _current=0.;v=_v;{ {
   if ( on ) {
     i = ( vc - v ) / ( 0.1 * rs ) ;
     i = i * 10.0 ;
     }
   else {
     i = 0.0 ;
     }
   }
 _current += i;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if EXTRACELLULAR
 _nd = _ml->_nodelist[_iml];
 if (_nd->_extnode) {
    _v = NODEV(_nd) +_nd->_extnode->_v[0];
 }else
#endif
 {
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 }
 _g = _nrn_current(_v + .001);
 	{ _rhs = _nrn_current(_v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) += _rhs;
  }else
#endif
  {
	NODERHS(_nd) += _rhs;
  }
  if (_nt->_nrn_fast_imem) { _nt->_nrn_fast_imem->_nrn_sav_rhs[_ni[_iml]] += _rhs; }
#if EXTRACELLULAR
 if (_nd->_extnode) {
   *_nd->_extnode->_rhs[0] += _rhs;
 }
#endif
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) -= _g;
  }else
#endif
  {
	NODED(_nd) -= _g;
  }
  if (_nt->_nrn_fast_imem) { _nt->_nrn_fast_imem->_nrn_sav_d[_ni[_iml]] -= _g; }
#if EXTRACELLULAR
 if (_nd->_extnode) {
   *_nd->_extnode->_d[0] += _g;
 }
#endif
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type){
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if EXTRACELLULAR
 _nd = _ml->_nodelist[_iml];
 if (_nd->_extnode) {
    _v = NODEV(_nd) +_nd->_extnode->_v[0];
 }else
#endif
 {
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 }
 v=_v;
{
 { error =  vstim();
 if(error){fprintf(stderr,"at line 76 in file VClamp.mod:\n        SOLVE vstim\n"); nrn_complain(_p); abort_run(error);}
 }}}

}

static void terminal(){}

static void _initlists() {
 int _i; static int _first = 1;
  if (!_first) return;
_first = 0;
}

#if NMODL_TEXT
static const char* nmodl_filename = "VClamp.mod";
static const char* nmodl_file_text = 
  "TITLE svclmp.mod\n"
  "\n"
  "COMMENT\n"
  "\n"
  "Single electrode Voltage clamp with three levels\n"
  "------------------------------------------------\n"
  "\n"
  "Series Resistance added; backards compatible, except parameters \n"
  "e0,vo0,vi0,gain,rstim,tau1,tau2 that no longer exist\n"
  "\n"
  "Clamp is on at time 0, and off at time dur[0]+dur[1]+dur[2]. When clamp is off\n"
  "the injected current is 0.  The clamp levels are amp[0], amp[1], amp[2].  i is\n"
  "the injected current, vc measures the control voltage) Do not insert several\n"
  "instances of this model at the same location in order to make level changes.\n"
  "That is equivalent to independent clamps and they will have incompatible\n"
  "internal state values.\n"
  "\n"
  "The electrical circuit for the clamp is exceedingly simple:\n"
  "\n"
  "        rs           Rin\n"
  "vc ---'\\/\\/`---o---'\\/\\/`---o\n"
  "               |            |\n"
  "               |____| |_____|\n"
  "                    | |\n"
  "                     Cm\n"
  "\n"
  "Note that since this is an electrode current model v refers to the internal\n"
  "potential which is equivalent to the membrane potential v when there is no\n"
  "extracellular membrane mechanism present but is v+vext when one is present. \n"
  "Also since i is an electrode current, positive values of i depolarize the\n"
  "cell. (Normally, positive membrane currents are outward and thus hyperpolarize\n"
  "the cell)\n"
  "\n"
  "ENDCOMMENT\n"
  "\n"
  "INDEPENDENT {t FROM 0 TO 1 WITH 1 (ms)}\n"
  "\n"
  "DEFINE NSTEP 3\n"
  "\n"
  "NEURON {\n"
  "        POINT_PROCESS TVClamp\n"
  "        ELECTRODE_CURRENT i\n"
  "        RANGE dur, amp, rs, vc, i   \n"
  "}\n"
  "\n"
  "UNITS {\n"
  "        (nA) = (nanoamp)\n"
  "        (mV) = (millivolt)\n"
  "        (uS) = (micromho)\n"
  "}\n"
  "\n"
  "\n"
  "PARAMETER {\n"
  "        v (mV)\n"
  "        rs = 3 (megohm)		: series resistance\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "        i (nA)\n"
  "        vc (mV)\n"
  "        ic (nA)\n"
  "        tc2 (ms)\n"
  "        tc3 (ms)\n"
  "	dur[NSTEP] (ms)\n"
  "	amp[NSTEP] (mV)\n"
  "        on\n"
  "}\n"
  "\n"
  "INITIAL {\n"
  "        tc2 = dur[0] + dur[1]\n"
  "        tc3 = tc2 + dur[2]\n"
  "        on = 0\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "        SOLVE vstim\n"
  "        if (on) {\n"
  "                i = (vc - v)/(0.1*rs)\n"
  "                i=i*10\n"
  "        }else{\n"
  "                i = 0\n"
  "        }\n"
  "}\n"
  "\n"
  "PROCEDURE vstim() {\n"
  "        on = 1\n"
  "        if (t < dur[0]) {\n"
  "                vc = amp[0]\n"
  "        }else if (t < tc2) {\n"
  "                vc =((amp[1]-amp[0])/dur[1])*(t-dur[0])+amp[0]  \n"
  "        }else if (t < tc3) {\n"
  "                vc = ((amp[2]-amp[1])/dur[2]) *(t-tc2)+amp[1]   \n"
  "        }else {\n"
  "                vc = amp[2]\n"
  "                on = 0\n"
  "        }\n"
  "        if (on) {\n"
  "        }else{\n"
  "                ic = 0\n"
  "        }\n"
  "        VERBATIM\n"
  "        return 0;\n"
  "        ENDVERBATIM\n"
  "}\n"
  ;
#endif
