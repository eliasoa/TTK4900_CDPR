function f = LINPROGGG(A_t, w, f_min, f_max)

c = [1 1 1 1]';

f = linprog(c,[],[],A_t,w,f_min*[1;1;1;1],f_max*[1;1;1;1]);

end