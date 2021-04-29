function [fitresult, gof] = fit_angles_vert_60in_cubic(pdoa_horz_60in, pdoa_vert_60in, angles_vert_deg_60in)
%CREATEFIT(PDOA_HORZ_60IN,PDOA_VERT_60IN,ANGLES_VERT_DEG_60IN)
%  Create a fit.
%
%  Data for 'fit_angles_vert_60in_cubic' fit:
%      X Input : pdoa_horz_60in
%      Y Input : pdoa_vert_60in
%      Z Output: angles_vert_deg_60in
%  Output:
%      fitresult : a fit object representing the fit.
%      gof : structure with goodness-of fit info.
%
%  See also FIT, CFIT, SFIT.

%  Auto-generated by MATLAB on 15-May-2020 17:16:48


%% Fit: 'fit_angles_vert_60in_cubic'.
[xData, yData, zData] = prepareSurfaceData( pdoa_horz_60in, pdoa_vert_60in, angles_vert_deg_60in );

% Set up fittype and options.
ft = fittype( 'poly33' );

% Fit model to data.
[fitresult, gof] = fit( [xData, yData], zData, ft, 'Normalize', 'on' );

% % Plot fit with data.
% figure( 'Name', 'fit_angles_vert_60in_cubic' );
% h = plot( fitresult, [xData, yData], zData );
% legend( h, 'fit_angles_vert_60in_cubic', 'angles_vert_deg_60in vs. pdoa_horz_60in, pdoa_vert_60in', 'Location', 'NorthEast', 'Interpreter', 'none' );
% % Label axes
% xlabel( 'pdoa_horz_60in', 'Interpreter', 'none' );
% ylabel( 'pdoa_vert_60in', 'Interpreter', 'none' );
% zlabel( 'angles_vert_deg_60in', 'Interpreter', 'none' );
% grid on
% view( 90.1, 5.4 );


