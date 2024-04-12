plot3((P3_r((N_init + 1):(prog_N - N_reset), 1)),(P3_r((N_init + 1):(prog_N - N_reset), 2)),(P3_r((N_init + 1):(prog_N - N_reset), 3)),'k-')
hold on
plot3((0),(0),(0),'k.','MarkerSize',6);
hold on
plot3([0 P1(1)],[0 P1(2)],[0 P1(3)],'r','LineWidth',2);
hold on
plot3([P1(1) P2(1)],[P1(2) P2(2)],[P1(3) P2(3)],'b','LineWidth',2);
hold on
plot3([P2(1) P3(1)],[P2(2) P3(2)],[P2(3) P3(3)],'g','LineWidth',2);
hold on
plot3(P3(1), P3(2), P3(3),'ro','MarkerSize',4);
if i >= (N_init + 1) && i <= (prog_N - N_reset)
   P3_plot = [P3_plot; P3'];
   plot3(P3_plot(:, 1), P3_plot(:, 2), P3_plot(:, 3),'r-'); 
   hold on
elseif i > (prog_N - N_reset)
   plot3(P3_plot(:, 1), P3_plot(:, 2), P3_plot(:, 3),'r-'); 
   hold on 
end
axis([-2*model.L1 2*model.L1 -1*model.L1 2*model.L1 0 3*model.L1]);
axis square
grid on
hold off
xlabel('x'); ylabel('y'); zlabel('z');