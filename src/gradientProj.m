function [z, iter] = gradientProj(H, h, zlb, zub, alpha, maxIter, tol)

    z = zeros(size(zlb));
    iter = 0;

    while iter < maxIter
        zPrev = z;
        grad = (H * z) + h;

        zTilde = z - (alpha * grad);

        z = min(max(zTilde,zlb), zub);

        if norm(z - zPrev) < tol
            break;
        end

        iter = iter + 1;

    end
end