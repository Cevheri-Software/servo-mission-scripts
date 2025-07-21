import asyncio
import math
import random
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

flight_altitude = 3
data_rate = 0.1
target_threshold = 0.5  # metre

mission_active = True


def calculate_yaw(dx, dy):
    yaw_radians = math.atan2(dy, dx)
    yaw_degrees = math.degrees(yaw_radians)
    yaw_normalized = (yaw_degrees + 360) % 360
    return yaw_normalized


def body_to_ned(x_body, y_body, yaw_deg):
    yaw_rad = math.radians(yaw_deg)
    north = x_body * math.cos(yaw_rad) - y_body * math.sin(yaw_rad)
    east = x_body * math.sin(yaw_rad) + y_body * math.cos(yaw_rad)
    return north, east


def ned_to_body(north, east, yaw_deg):
    """
    NED koordinatlarından (North, East) body koordinatlarına (x_body, y_body) çevirir.
    yaw_deg: Aracın yönü (derece cinsinden, saat yönünün tersine pozitif)
    """
    yaw_rad = math.radians(yaw_deg)
    x_body = north * math.cos(yaw_rad) + east * math.sin(yaw_rad)
    y_body = -north * math.sin(yaw_rad) + east * math.cos(yaw_rad)
    return x_body, y_body


async def generate_target_location():
    while mission_active:
        dx = 3  # İleri hareket
        dy = 0  # Sağ/sol
        distance = math.sqrt(dx ** 2 + dy ** 2)
        await asyncio.sleep(data_rate)
        yield dx, dy, distance


async def connect_drone():
    drone = System()
    #await drone.connect(system_address='udp://:14540')
    await drone.connect(system_address="serial:///dev/ttyUSB0:57600")

    async for state in drone.core.connection_state():
        if state.is_connected:
            print("✅ Drone connected!")
            break
    return drone


async def takeoff(drone):
    await drone.telemetry.set_rate_position_velocity_ned(10.0)
    await drone.telemetry.set_rate_attitude_euler(10.0)
    await drone.action.hold()
    await asyncio.sleep(1)
    await drone.action.arm()
    await drone.action.takeoff()
    await asyncio.sleep(2)
    print("🚀 Drone takeoff completed!")


async def enter_offboard_mode(drone):
    try:
        await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -flight_altitude, 0))
        await drone.offboard.start()
        print("🟢 Offboard mode activated!")
        return True
    except OffboardError as err:
        print(f"❌ Offboard error: {err}")
        return False


async def get_current_position(drone):
    async for pos in drone.telemetry.position_velocity_ned():
        return pos.position


async def get_current_yaw(drone):
    async for attitude in drone.telemetry.attitude_euler():
        return attitude.yaw_deg


async def execute_mission(drone):
    global mission_active
    last_known_yaw = 0.0  # fallback

    try:
        async for dx, dy, _ in generate_target_location():
            if not mission_active:
                print("🛑 Mission terminated")
                break

            # Mevcut pozisyon ve yaw'ı al
            current_pos = await get_current_position(drone)
            current_yaw = await get_current_yaw(drone)

            if current_yaw is None:
                current_yaw = last_known_yaw
                print(f"⚠ Using last known yaw: {current_yaw:.2f}°")
            else:
                last_known_yaw = current_yaw

            # Yeni hedef pozisyonu hesapla (mevcut pozisyondan başlayarak)
            north_offset, east_offset = body_to_ned(dx, dy, current_yaw)
            target_north = current_pos.north_m + north_offset
            target_east = current_pos.east_m + east_offset

            # Hedef yaw'ı hesapla
            relative_yaw = calculate_yaw(dx, dy)
            target_yaw = (current_yaw + relative_yaw) % 360

            print(f"\n📦 Gelen veri: dx={dx:.2f} m, dy={dy:.2f} m")
            print(f"📍 Mevcut pozisyon: north={current_pos.north_m:.2f}, east={current_pos.east_m:.2f}")
            print(f"🧭 Mevcut yaw: {current_yaw:.2f}°, hedef yaw: {target_yaw:.2f}°")
            print(f"🎯 Yeni hedef pozisyon: north={target_north:.2f}, east={target_east:.2f}")

            reached = False
            while not reached and mission_active:
                try:
                    await drone.offboard.set_position_ned(
                        PositionNedYaw(target_north, target_east, -flight_altitude, target_yaw)
                    )
                    await asyncio.sleep(data_rate)

                    # Güncel pozisyonu al
                    pos = await get_current_position(drone)
                    current_yaw_check = await get_current_yaw(drone)

                    # Hedefe olan mesafeyi hesapla
                    dn = target_north - pos.north_m
                    de = target_east - pos.east_m
                    dist = math.sqrt(dn ** 2 + de ** 2)

                    # Body koordinatlarına çevir (görselleştirme için)
                    if current_yaw_check is not None:
                        dx_body, dy_body = ned_to_body(dn, de, current_yaw_check)
                    else:
                        dx_body, dy_body = ned_to_body(dn, de, current_yaw)

                    if dist < target_threshold:
                        print(f"✅ Hedefe ulaşıldı! Mesafe: {dist:.2f} m")
                        reached = True
                    else:
                        print(
                            f"⏳ Hedefe yaklaşılıyor: İleri:{dx_body:.2f} m Sağ:{dy_body:.2f} m (Toplam: {dist:.2f} m)")

                except OffboardError as offboard_err:
                    print(f"\n❌ Offboard control error: {offboard_err}")
                    mission_active = False
                    break
                except Exception as e:
                    print(f"\n❌ Hedefe gitme hatası: {e}")
                    mission_active = False
                    break

    except Exception as e:
        print(f"\n❌ Görev yürütme hatası: {e}")
    finally:
        mission_active = False


async def emergency_landing(drone):
    global mission_active
    mission_active = False
    try:
        print('\n🚨 Acil iniş başlatılıyor')
        await drone.offboard.stop()
        await drone.action.land()
        print('✅ Acil iniş tamamlandı')
    except Exception as e:
        print(f'❌ Acil iniş başarısız: {e}')


async def main():
    global mission_active

    try:
        drone = await connect_drone()
        await takeoff(drone)

        if not await enter_offboard_mode(drone):
            print("Programdan çıkılıyor...")
            return

        await execute_mission(drone)

    except KeyboardInterrupt:
        print('\n🛑 Görev kullanıcı tarafından durduruldu')
        if 'drone' in locals():
            await emergency_landing(drone)
    except Exception as e:
        print(f'\n❌ Kritik hata: {e}')
        if 'drone' in locals():
            await emergency_landing(drone)
    finally:
        mission_active = False


if _name_ == "_main_":
    asyncio.run(main())