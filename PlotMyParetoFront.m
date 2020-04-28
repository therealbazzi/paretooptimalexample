clc
clear all

x0 = [-1,-1,-1];
N = 100;
fun = @vp;

[x1_star,fx1] = fminsearch(@(x)pickindex(x,1),x0);
[x2_star,fx2] = fminsearch(@(x)pickindex(x,2),x0);
goal = [fx1,fx2];

for k = 0:N
   alpha = k/N;
   weight = [alpha,1-alpha];
   [temp,f(k+1,:)]=fgoalattain(fun,x0,goal,weight);
end

figure
plot(f(:,1),f(:,2),'ko')
xlabel('f_1')
ylabel('f_2')


A = [1 2 3;2 1 5;1 2 2];
b = [1;2;3];
k=1;
for alpha = 0:0.001:5
   x = inv(A'*A + alpha*eye(3))*A'*b;
   F1(k) = norm(A*x - b,2)^2;
   F2(k) = norm(x,2)^2;
   k=k+1;
end
hold on
plot(F1,F2,'r')

