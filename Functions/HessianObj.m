function Hessian_x = HessianObj(in1,f_min,f_max,f_ref,p,c,b,epsilon)
%HessianObj
%    Hessian_x = HessianObj(IN1,F_MIN,F_MAX,F_REF,P,C,B,EPSILON)

%    This function was generated by the Symbolic Math Toolbox version 23.2.
%    15-Apr-2024 17:08:09

f1 = in1(1,:);
f2 = in1(2,:);
f3 = in1(3,:);
f4 = in1(4,:);
s1 = in1(5,:);
s2 = in1(6,:);
s3 = in1(7,:);
t2 = s1.^2;
t3 = s2.^2;
t4 = s3.^2;
t5 = -f_min;
t6 = -f_max;
t7 = -f_ref;
t8 = -p;
t9 = p-1.0;
t14 = f_min./2.0;
t15 = f_max./2.0;
t10 = t9-1.0;
t11 = epsilon+t2;
t12 = epsilon+t3;
t13 = epsilon+t4;
t16 = -t14;
t17 = t15+t16;
t18 = t17.^t8;
mt1 = [c.*1.0./(f1+t5).^2+c.*1.0./(f1+t6).^2+p.*t9.*t18.*(f1+t7).^t10,0.0,0.0,0.0,0.0,0.0,0.0,0.0,c.*1.0./(f2+t5).^2+c.*1.0./(f2+t6).^2+p.*t9.*t18.*(f2+t7).^t10,0.0,0.0,0.0,0.0,0.0,0.0,0.0,c.*1.0./(f3+t5).^2+c.*1.0./(f3+t6).^2+p.*t9.*t18.*(f3+t7).^t10,0.0,0.0,0.0,0.0,0.0,0.0,0.0,c.*1.0./(f4+t5).^2+c.*1.0./(f4+t6).^2+p.*t9.*t18.*(f4+t7).^t10,0.0,0.0,0.0,0.0,0.0,0.0,0.0,b.*1.0./sqrt(t11)-b.*t2.*1.0./t11.^(3.0./2.0)+2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,b.*1.0./sqrt(t12)-b.*t3.*1.0./t12.^(3.0./2.0)+2.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0];
mt2 = [b.*1.0./sqrt(t13)-b.*t4.*1.0./t13.^(3.0./2.0)+2.0];
Hessian_x = reshape([mt1,mt2],7,7);
end