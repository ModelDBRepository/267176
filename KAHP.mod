
 COMMENT
 
 KAHP.mod
 
 Calcium-dependent potassium channel responsible for mAHP in motoneurons
 Simplified calcium channel that provides Ca for the KCa conductance is included.
 Originally taken from Powers et al, 2012.
 	
 ENDCOMMENT

 NEURON {
 	SUFFIX KAHP
 	USEION k READ ek WRITE ik
 	USEION ca READ eca WRITE ica
 	RANGE  gkcamax,gcamax,ik,cai,ica,taur
 	GLOBAL bKCa 
 }

 
 UNITS {
 	(mA) = (milliamp)
 	(mV) = (millivolt)
 	(S) = (siemens)
 	(um) = (micron)
 	(molar) = (1/liter)			: moles do not appear in units
 	(mM)	= (millimolar)
 	(msM)	= (ms mM)
 	FARADAY = (faraday) (coulomb)
 } 
 
 PARAMETER {
 	gkcamax = 0.05   	(S/cm2)	
	gcamax = 3e-5		(S/cm2)
   
  cainf=0.0001		(mM)
 	taur	= 10		(ms)		: rate of calcium removal
 
 	bKCa   = 0.1	  : max deact rate 
  B=-17.024       :scaling constant
 	celsius		(degC)
 } 
 
 
 ASSIGNED {
 	ik 		(mA/cm2)
 	v 		(mV)
	ica 	(mA/cm2)
 	ek		(mV)
	eca		(mV)
 	qinf
 	qtau 		(ms)
  minfca
  mtauca  (ms)
  hinfca
  htauca  (ms) 
 }
  
 
 STATE {
 mca
 hca 
 q 
 cai (mM)
}
 
 INITIAL { 
	cai=cainf
 	rates(cai)
	mcarate(v)
 	q = qinf
	mca=minfca
  hca=hinfca
 }
 
 BREAKPOINT {
         SOLVE states METHOD cnexp
	ica = gcamax*mca^2*hca*(v - eca)   :CaN
 	ik =  gkcamax *q* (v - ek)
 } 
 

DERIVATIVE states { 
	 
         rates(cai) 
 	       cai' = B*ica-cai/taur  
         q' = (qinf-q)/qtau
         mcarate(v)    
         mca' = (minfca - mca)/mtauca
         hca' = (hinfca - hca)/htauca
}
PROCEDURE rates(cai(mM)) {  LOCAL alphaq,betaq
					 
         alphaq=cai           :related to [Ca]in
         betaq = bKCa
         qtau = 1/(alphaq+betaq)
         qinf = alphaq*qtau
				 
 }

PROCEDURE mcarate(v (mV)) {
   LOCAL alphacam,betacam ,  alphacah,betacah
   alphacam=0.25/(1+exp((v+20)/-5))
   betacam=0.25/(1+exp((v+20)/5))
   minfca = alphacam/(alphacam+betacam) 
   mtauca= 1/(alphacam+betacam)
   
   alphacah=0.025/(1-exp((v+35)/-5))
   betacah =0.025/(1-exp((v+35)/5))
   
   hinfca= alphacah/(alphacah+betacah)
   htauca= 1/(alphacah+betacah)
    
}
