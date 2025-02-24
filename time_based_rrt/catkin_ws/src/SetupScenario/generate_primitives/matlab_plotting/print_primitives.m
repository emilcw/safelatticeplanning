% Printing stuff
number_of_groups = 8;
num_prim_in_group = zeros(1,5);

for i=1:number_of_groups
    filePath = strcat('../config/lattice_group', num2str(i) ,'.txt');
    num = dlmread(filePath,'',[0 0 0 0]);
    num_prim_in_group(1,i) = num(1);
end

header_rows = 11;
start_vx = 0;
start_vy = 0.;
start_vz = 1;
figure(1)
clf;

for i=1:number_of_groups
    for n=0:num_prim_in_group(i)-1

      Trajpath = strcat('../primitives/primitive_group_', ...
                        num2str(i) ,'_id_', num2str(n),'.txt');

      Trajectory = load_trajectory(Trajpath,header_rows);

      hold on
      if(abs(Trajectory.vx(1) - start_vx) < 0.1 && ...
         abs(Trajectory.vy(1) - start_vy) < 0.1 && ...
         abs(Trajectory.vz(1) - start_vz) < 0.1)
        plot3(Trajectory.x,Trajectory.y,Trajectory.z-2)
      end

    end
end

axis([-1 1 -1 1 -1 1])
grid on
view([90, 70, 50])
xlabel('x [m]','interpreter','latex')
ylabel('y [m]','interpreter','latex')
zlabel('z [m]','interpreter','latex')

%%

PLOT_DOTS = 1;

Trajpath = strcat('../primitives/primitive_group_2_id_13.txt');
header_rows = 11;
Trajectory = load_trajectory(Trajpath,header_rows);

figure(2); clf;
subplot(3,3,1); hold on;
if PLOT_DOTS, plot(Trajectory.t,Trajectory.x, '.k'); end
plot(Trajectory.t,Trajectory.x, 'b');
axis([0 Trajectory.t(end) -1.2 1.2])
ylabel('$x$[m]','interpreter','latex')
xlabel('t[s]','interpreter','latex')
grid
subplot(3,3,2); hold on;
if PLOT_DOTS, plot(Trajectory.t,Trajectory.y,'.k'); end
plot(Trajectory.t,Trajectory.y,'b');
axis([0 Trajectory.t(end) -1.2 1.2])
ylabel('$y$[m]','interpreter','latex')
xlabel('t[s]','interpreter','latex')
grid
subplot(3,3,3); hold on;
if PLOT_DOTS, plot(Trajectory.t,Trajectory.z-2,'.k'); end
plot(Trajectory.t,Trajectory.z-2,'b');
axis([0 Trajectory.t(end) -1.2 1.2])
ylabel('$z$[m]','interpreter','latex')
xlabel('t[s]','interpreter','latex')
grid
subplot(3,3,4); hold on;
if PLOT_DOTS, plot(Trajectory.t,Trajectory.vx,'.k'); end
plot(Trajectory.t,Trajectory.vx,'b');
axis([0 Trajectory.t(end) -1.2 1.2])
ylabel('$v_x$[m/s]','interpreter','latex')
xlabel('t[s]','interpreter','latex')
grid
subplot(3,3,5); hold on;
if PLOT_DOTS, plot(Trajectory.t,Trajectory.vy,'.k'); end
plot(Trajectory.t,Trajectory.vy,'b');
axis([0 Trajectory.t(end) -1.2 1.2])
ylabel('$v_y$[m/s]','interpreter','latex')
xlabel('t[s]','interpreter','latex')
grid
subplot(3,3,6); hold on;
if PLOT_DOTS, plot(Trajectory.t,Trajectory.vz,'.k'); end
plot(Trajectory.t,Trajectory.vz,'b');
axis([0 Trajectory.t(end) -1.2 1.2])
ylabel('$v_z$[m/s]','interpreter','latex')
xlabel('t[s]','interpreter','latex')
grid
subplot(3,3,7); hold on;
if PLOT_DOTS, plot(Trajectory.t,Trajectory.roll,'.k'); end
plot(Trajectory.t,Trajectory.roll,'b');
axis([0 Trajectory.t(end) -0.2 0.2])
ylabel('$\phi$ [rad]','interpreter','latex')
xlabel('t[s]','interpreter','latex')
grid
subplot(3,3,8); hold on;
if PLOT_DOTS, plot(Trajectory.t,Trajectory.pitch,'.k'); end
plot(Trajectory.t,Trajectory.pitch,'b');
axis([0 Trajectory.t(end) -0.2 0.2])
ylabel('$\theta$ [rad]','interpreter','latex')
xlabel('t[s]','interpreter','latex')
grid
subplot(3,3,9); hold on;
if PLOT_DOTS, plot(Trajectory.t,Trajectory.thrust,'.k'); end
plot(Trajectory.t,Trajectory.thrust,'b');
axis([0 Trajectory.t(end) 8 12])
ylabel('T [m/$s^2$]','interpreter','latex')
xlabel('t[s]','interpreter','latex')
grid

