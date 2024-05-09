function f = FrictionModel(ODriveNumber,v)
v_l = 0.5;
c0 = [ 0.00118181818181995,  -0.00205151515152902,	0.0587848484848724];
% c0 = [0.00118181818183111	-0.00205151515157875	0.0987848484849043];
% c0 = [0.00118181818182805	-0.00205151515156510	0.0887848484848948];
% c0 = [0.00118181818183111	-0.00205151515157875	0.0987848484849043];
% c0 = [0.00118181818183488	-0.00205151515158775	0.118784848484890];
% c0 = [0.00118181818182789	-0.00205151515156621	0.148784848484901];
% c0 = [0.00118181818183455	-0.00205151515159097	-0.141215151515094];
% c1 = [ 0.000969696969704059, -0.00149696969700408,	0.0602969696969962];
% c1 = [-0.000606060606050063,  0.00579393939388689,	0.0604727272727768];
% c1 = [-0.000606060606051839,	0.00579393939389528,	0.0654727272727664];

c1 = [-0.000606060606041070	0.00579393939384920	0.0754727272727929];
c1_in = [-0.000242424242411388,	0.00422424242418717,	0.0589757575757854];
c1_out = [-0.00103030303029028,	0.00867636363630303,	0.0939503030303555];
% c1 = [-0.000606060606050729	0.00579393939389378	0.0954727272727570];
% c1 = [-0.000242424242413164	0.00422424242418912	0.0389757575758027];
c2 = [ 3.03030303101526e-05,  0.00346363636360586,	0.0467696969697113];
% c2 = [3.03030303152041e-05	0.00346363636357605	0.0767696969697521];
c3 = [-0.000727272727265027,  0.00773939393935047,	0.0427939393939897];
% c3 = [-0.000727272727258477	0.00773939393932627	0.0727939393939971];
% c3 = [-0.000727272727265804	0.00773939393935769	0.102793939393978];
% c3 = [-0.000727272727264694	0.00773939393935352	0.132793939393976];
% c3 = [-0.000727272727252148	0.00773939393929746	0.192793939394012];

b0 = [0,	0.175189393939453,	-0.00118181818181998,	-0.236321212121310,	 0.00472727272727985];
% b0 = [0	0.295189393939508	-0.00118181818183113	-0.396321212121448	0.00472727272732448];
% b0 = [0	0.265189393939490	-0.00118181818182808	-0.356321212121407	0.00472727272731227];
% b0 = [0	0.295189393939508	-0.00118181818183113	-0.396321212121448	0.00472727272732448];
% b0 = [0	0.355189393939458	-0.00118181818183491	-0.476321212121394	0.00472727272733958];
% b0 = [0	0.445189393939509	-0.00118181818182780	-0.596321212121433	0.00472727272731138];
% b0 = [0	-0.424810606060497	-0.00118181818183455	0.563678787878541	0.00472727272733819];
% b1 = [0,	0.180121212121263,	-0.000969696969704029,	-0.242157575757689,	 0.00387878787881618];
% b1 = [0,	0.180969696969735,	 0.000242424242411388,	-0.235660606060730,	-0.000969696969645550];
% b1 = [0,	0.186757575757680,	 0.000606060606050007,	-0.241284848485057,	-0.00242424242420014];
% b1 = [0	0.201757575757656	0.000606060606051837	-0.261284848485014	-0.00242424242420735];
b1 = [0	0.235559090909163	0.146424848484926	-0.306003636363810	-0.289670303030437]; % med asymmetrisk
% b1 = [0	0.231757575757697	0.000606060606041126	-0.301284848485130	-0.00242424242416439];
% b1 = [0	0.291757575757627	0.000606060606050729	-0.381284848484977	-0.00242424242420292];
% b1 = [0	0.120969696969787	0.000242424242413164	-0.155660606060798	-0.000969696969652656];
b2 = [0,	0.143795454545473,	-3.03030303101526e-05,	-0.187109090909156,	 0.000121212121240610];
% b2 = [0	0.233795454545569	-3.03030303152319e-05	-0.307109090909323	0.000121212121260872];
b3 = [0,	0.135575757575871,	 0.000727272727265027,	-0.170448484848694,	-0.00290909090906011];
% b3 = [0	0.225575757575874	0.000727272727258421	-0.290448484848730	-0.00290909090903380];
% b3 = [0	0.315575757575843	0.000727272727265638	-0.410448484848648	-0.00290909090906288];
% b3 = [0	0.405575757575832	0.000727272727264861	-0.530448484848637	-0.00290909090905911];
% b3 = [0	0.585575757575894	0.000727272727252204	-0.770448484848796	-0.00290909090900882];


switch ODriveNumber
    case 0
        if v >= v_l
            f = c0(1)*v^2 + c0(2)*v + c0(3);
        elseif v < -v_l
            f = - (c0(1)*abs(v)^2 + c0(2)*abs(v) + c0(3));
        elseif (-v_l <= v) && (v < v_l)
            f = b0(1) + b0(2)*v.^1 + b0(3)*v.^2 + b0(4)*v.^3 + b0(5)*v.^4;
        end

    case 1
        if v >= v_l
            % f = c1(1)*v^2 + c1(2)*v + c1(3);
            f = c1_out(1)*v^2 + c1_out(2)*v + c1_out(3);
        elseif v < -v_l
            % f = - (c1(1)*abs(v)^2 + c1(2)*abs(v) + c1(3));
            f = -(c1_in(1)*abs(v)^2 + c1_in(2)*abs(v) + c1_in(3));
        elseif (-v_l <= v) && (v < v_l)
            f = (b1(1) + b1(2)*v^1 + b1(3)*v.^2 + b1(4)*v^3 + b1(5)*v^4);
        end

    case 2
        if v >= v_l
            f = c2(1)*v.^2 + c2(2)*v + c2(3);
        elseif v < -v_l
            f = - (c2(1)*abs(v)^2 + c2(2)*abs(v) + c2(3));
        elseif (-v_l <= v) && (v < v_l)
            f = b2(1) + b2(2)*v.^1 + b2(3)*v.^2 + b2(4)*v.^3 + b2(5)*v.^4;
        end

    case 3
        v = fix(v*10^2)/10^2;
        if v >= v_l
            f = c3(1)*v.^2 + c3(2)*v + c3(3);
        elseif v < -v_l
            f = - (c3(1)*abs(v)^2 + c3(2)*abs(v) + c3(3));
        elseif (-v_l <= v) && (v < v_l)
            f = b3(1) + b3(2)*v^1 + b3(3)*v^2 + b3(4)*v^3 + b3(5)*v^4;
        end
end
end