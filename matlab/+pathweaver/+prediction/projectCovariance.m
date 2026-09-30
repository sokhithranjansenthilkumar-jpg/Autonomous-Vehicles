function covariance = projectCovariance(covariance, eigenvalueFloor)
%PROJECTCOVARIANCE Symmetrize and project a 2-D covariance to positive definite.
assert(isequal(size(covariance), [2 2]) && all(isfinite(covariance), 'all'), ...
    'PathWeaver:InvalidCovariance', 'Covariance must be a finite 2-by-2 matrix.');
covariance = (covariance + covariance')/2;
[vectors, values] = eig(covariance);
values = diag(max(diag(values), eigenvalueFloor));
covariance = vectors*values*vectors';
covariance = (covariance + covariance')/2;
end
