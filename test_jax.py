import jax
import jax.numpy as jnp

# 检查 JAX 是否可以使用 GPU
print("JAX is using GPU:" if jax.devices()[0].device_kind == 'Gpu' else "JAX is not using GPU")

# 测试矩阵运算
def test_matrix_operations():
    print("\nTesting matrix operations...")
    A = jnp.array([[1.0, 2.0], [3.0, 4.0]])
    B = jnp.array([[5.0, 6.0], [7.0, 8.0]])
    C = jnp.dot(A, B)
    print("Matrix A:\n", A)
    print("Matrix B:\n", B)
    print("Matrix multiplication A * B:\n", C)

# 测试自动微分
def test_autograd():
    print("\nTesting autograd...")
    def f(x):
        return x ** 2 + 3 * x + 2

    df_dx = jax.grad(f)
    x = 2.0
    print(f"f(x) = x^2 + 3x + 2")
    print(f"f'({x}) = {df_dx(x)}")

if __name__ == "__main__":
    # 打印 JAX 版本信息
    print(f"JAX version: {jax.__version__}")
    print(f"JAX devices: {jax.devices()}")

    # 测试 JAX 基本功能
    test_matrix_operations()
    test_autograd()
