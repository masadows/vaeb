from .scenarios.scenarios import (
    StationaryVehicleTest,
    LeadVehicleBrakingTest,
    SuicidalPedestrianTest,
    FalsePositiveTest,
    PedestrianCrossingTest,
    OncomingTrafficTest,
    CutInTest,
    ObstructedPedestrianTest,
    ObstacleOnCurveTest,
)
from vaeb.utils.vision import load_models
from vaeb.env.test import CarlaTestManager
import time


def main():
    test_mgr = CarlaTestManager()
    models = load_models()

    # test_suite = [
    #     FalsePositiveTest(),
    #     LeadVehicleBrakingTest(start_distance=30, target_speed=60),
    #     SuicidalPedestrianTest(
    #         trigger_distance=40,
    #         spawn_distance=50,
    #         stop_distance=2.5,
    #         speed=40,
    #         max_duration=7,
    #     ),
    #     SuicidalPedestrianTest(
    #         trigger_distance=20, stop_distance=2.5, speed=30, max_duration=7
    #     ),
    #     PedestrianCrossingTest(trigger_distance=20.0, max_duration=5),
    #     PedestrianCrossingTest(trigger_distance=15.0, max_duration=5),
    #     PedestrianCrossingTest(trigger_distance=10.0, max_duration=5),
    #     OncomingTrafficTest(
    #         lateral_offset=3.0,
    #         start_distance=60,
    #         ego_speed=40,
    #         oncoming_speed=40,
    #         max_duration=15,
    #         should_stop=False,
    #     ),
    #     OncomingTrafficTest(
    #         lateral_offset=2.0,
    #         start_distance=60,
    #         ego_speed=30,
    #         oncoming_speed=30,
    #         stop_distance=0,
    #         should_stop=True,
    #         max_duration=15,
    #     ),
    #     OncomingTrafficTest(
    #         lateral_offset=0.0,
    #         start_distance=60,
    #         ego_speed=30,
    #         oncoming_speed=30,
    #         stop_distance=9,
    #         should_stop=True,
    #         max_duration=15,
    #     ),
    #     CutInTest(
    #         lateral_offset=3.5, start_distance=15.0, speed=40.0, brake_after_cutin=True
    #     ),
    #     CutInTest(
    #         lateral_offset=3.5, start_distance=10.0, speed=30.0, brake_after_cutin=True
    #     ),
    #     ObstructedPedestrianTest(trigger_distance=15.0, speed=30.0),
    #     ObstructedPedestrianTest(trigger_distance=18.0, speed=40.0),
    #     ObstacleOnCurveTest(distance=40.0, speed=30.0),
    #     ObstacleOnCurveTest(distance=60.0, speed=40.0),
    # ]

    # for d in [30, 40, 50]:
    #     test_suite.append(StationaryVehicleTest(distance=d))

    test_suite = [
        FalsePositiveTest(),
        LeadVehicleBrakingTest(start_distance=10, target_speed=40),
        LeadVehicleBrakingTest(start_distance=10, target_speed=20),
        LeadVehicleBrakingTest(start_distance=20, target_speed=80),
        LeadVehicleBrakingTest(start_distance=15, target_speed=50),
        LeadVehicleBrakingTest(start_distance=25, target_speed=100),
        StationaryVehicleTest(distance=10, target_speed=10),
        StationaryVehicleTest(distance=20, target_speed=20),
        StationaryVehicleTest(distance=30, target_speed=50),
        StationaryVehicleTest(distance=40, target_speed=40),
        StationaryVehicleTest(distance=50, target_speed=50),
        StationaryVehicleTest(distance=30, target_speed=50),
        StationaryVehicleTest(distance=40, target_speed=60),
        StationaryVehicleTest(distance=80, target_speed=70),
        StationaryVehicleTest(distance=100, target_speed=90),
        StationaryVehicleTest(distance=60, target_speed=100),
        StationaryVehicleTest(distance=120, target_speed=120),
        SuicidalPedestrianTest(
            trigger_distance=40,
            spawn_distance=50,
            stop_distance=2.5,
            speed=40,
            max_duration=7,
        ),
        SuicidalPedestrianTest(
            trigger_distance=15,
            spawn_distance=20,
            stop_distance=2.5,
            speed=20,
            max_duration=7,
        ),
        SuicidalPedestrianTest(
            trigger_distance=25,
            spawn_distance=30,
            stop_distance=2.5,
            speed=60,
            max_duration=6,
        ),
        PedestrianCrossingTest(trigger_distance=12.0, max_duration=8),
        PedestrianCrossingTest(trigger_distance=20.0, max_duration=6),
        PedestrianCrossingTest(trigger_distance=10.0, max_duration=5),
        PedestrianCrossingTest(trigger_distance=15.0, max_duration=8),
        PedestrianCrossingTest(trigger_distance=17.0, max_duration=8),
        ObstructedPedestrianTest(trigger_distance=10.0, speed=20.0),
        ObstructedPedestrianTest(trigger_distance=12.0, speed=30.0),
        ObstructedPedestrianTest(trigger_distance=15.0, speed=30.0),
        ObstructedPedestrianTest(trigger_distance=18.0, speed=40.0),
        OncomingTrafficTest(
            lateral_offset=3.5,
            start_distance=60,
            ego_speed=40,
            oncoming_speed=40,
            max_duration=10,
        ),
        OncomingTrafficTest(
            lateral_offset=3.5,
            start_distance=100,
            ego_speed=60,
            oncoming_speed=60,
            max_duration=10,
        ),
        OncomingTrafficTest(
            lateral_offset=1.5,
            start_distance=60,
            ego_speed=30,
            oncoming_speed=30,
            should_stop=True,
            stop_distance=7,
            max_duration=10,
        ),
        OncomingTrafficTest(
            lateral_offset=1.0,
            start_distance=80,
            ego_speed=60,
            oncoming_speed=60,
            should_stop=True,
            stop_distance=7,
            max_duration=10,
        ),
        OncomingTrafficTest(
            lateral_offset=0.0,
            start_distance=100,
            ego_speed=50,
            oncoming_speed=50,
            stop_distance=10,
            should_stop=True,
            max_duration=15,
        ),
        CutInTest(
            lateral_offset=3.5, start_distance=15.0, speed=30.0, brake_after_cutin=True
        ),
        CutInTest(
            lateral_offset=3.5, start_distance=8.0, speed=30.0, brake_after_cutin=True
        ),
        CutInTest(
            lateral_offset=3.5, start_distance=5.0, speed=50.0, brake_after_cutin=False
        ),
        CutInTest(
            lateral_offset=3.5, start_distance=5.0, speed=30.0, brake_after_cutin=True
        ),
        CutInTest(
            lateral_offset=3.5, start_distance=10.0, speed=25.0, brake_after_cutin=True
        ),
        ObstacleOnCurveTest(distance=40.0, speed=30.0),
        ObstacleOnCurveTest(distance=60.0, speed=50.0),
        ObstacleOnCurveTest(distance=25.0, speed=30.0),
        ObstacleOnCurveTest(distance=60.0, speed=30.0),
    ]

    results = []
    try:
        for _ in test_mgr.change_weather():
            for test in test_suite:
                try:
                    res = test.run(test_mgr, models)
                    results.append(res)
                    time.sleep(2)
                except Exception as e:
                    print(f"Błąd krytyczny w teście {test.name}: {e}")
                    test_mgr.reset()
                    test_mgr.cleanup()
                finally:
                    test_mgr.reset()
                    test_mgr.cleanup()
                    test_mgr.destroy_all_spawned_actors()
    except KeyboardInterrupt:
        print("Przerwano testy.")
        test_mgr.reset()
        test_mgr.cleanup()

    print("\n" + "=" * 50)
    print("RAPORT KOŃCOWY")
    print("=" * 50)

    try:
        import pandas as pd

        df = pd.DataFrame(results)
        print(
            df[
                ["name", "result", "min_distance", "brake_reaction_time", "stopped"]
            ].to_string()
        )
        # df.to_csv("summary.csv", index=True)
    except ImportError:
        for r in results:
            print(
                f"{r['name']:<30} | {r['result']:<15} | MinDist: {r['min_distance']:.2f} | ReactionTime: {r['brake_reaction_time']:.2f}"
            )


if __name__ == "__main__":
    main()
