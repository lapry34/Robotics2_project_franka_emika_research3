function q_dot = reduced_gradient(r_dot, q_list, a_list, b_list, J, grad_H)
    % r_dot: desired velocity
    % q_list: list of joint angles
    % a_list: list of elements of q that are in A
    % b_list: list of elements of q that are in B
    % J: Jacobian matrix
    % grad_H: gradient of the objective function H(q) wrt q

    % TODO: extract the elements of q that are in A and B
    q_a = q_list(a_list);
    q_b = q_list(b_list);

    % order also grad_H to match the order of q_list
    grad_H = grad_H([a_list, b_list]);

    %extract the columns of the jacobian related to a_list and b_list and put it in J_a and J_b
    J_a = J(:, a_list);
    J_b = J(:, b_list);

    %disp(J)

    %disp("J_a and J_b")
    %disp(J_a)
    %disp(J_b)
    %disp("----")

    %M is the rows of J, N is the columns of J
    [M, N] = size(J);

    %disp("size of J")
    %fprintf("M: %d, N: %d\n", M, N)

    % create identity of size M-N
    id = eye(N-M);

    J_a_inv = inv(J_a); %pinv??? damped?? forse al midterm chi lo saaaa

    %disp("J_a_inv")
    %disp(J_a_inv)

    %disp(zeros(size(J_a_inv, 1), N-M))


    J_inv = [J_a_inv; zeros(N-M, size(J_a_inv, 1))];

    %disp("J_inv")
    %disp(J_inv)

    F = [-J_a_inv*J_b; id];

    %disp(F)
    FF = F*F';

    %disp(FF)

    q_dot = J_inv * r_dot + FF * grad_H;

    %reorder q_dot to match the original order
    %extract the maximum value in a_list and b_list
    max_val = max([a_list, b_list]);
    %generate a list from 1 to max_val 
    q_dot = q_dot([1:max_val]);


end