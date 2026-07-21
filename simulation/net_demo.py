"""
扯网拦截备份入口。
单独启动，不影响原有撞击拦截模式。
"""
import argparse

from simulation.main import run_demo


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--seed', type=int, default=42)
    parser.add_argument('--scene-km', type=float, default=5.0, help='场景尺度: 1/2/3/4/5/10 km')
    parser.add_argument('--source', default='auto', choices=['auto', 'redis', 'demo'])
    parser.add_argument('--demo-case', default='barrier-single', choices=['net-single', 'barrier-single'])
    parser.add_argument('--redis-host', default='127.0.0.1')
    parser.add_argument('--redis-port', type=int, default=6379)
    parser.add_argument('--redis-db', type=int, default=0)
    parser.add_argument('--test-wav', type=str, default=None)
    args = parser.parse_args()

    run_demo(
        seed=args.seed,
        test_wav=args.test_wav,
        scene_km=args.scene_km,
        source=args.source,
        redis_host=args.redis_host,
        redis_port=args.redis_port,
        redis_db=args.redis_db,
        intercept_mode='net',
        demo_case=args.demo_case,
    )


if __name__ == '__main__':
    main()
