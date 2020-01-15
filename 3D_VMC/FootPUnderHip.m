function y=FootPUnderHip(u)

LF_Q=u(1:3);RF_Q=u(4:6);
RH_Q=u(7:9);LH_Q=u(10:12);
LF_P=Cal_LiftOff_footP(LF_Q);
RF_P=Cal_LiftOff_footP(RF_Q);
RH_P=Cal_LiftOff_footP(RH_Q);
LH_P=Cal_LiftOff_footP(LH_Q);
y=[LF_P;RF_P;RH_P;LH_P];
