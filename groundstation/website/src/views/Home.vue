<!-- eslint-disable vue/multi-word-component-names -->
<script lang="ts">
import { ref, inject, defineComponent } from "vue";
import { socketProvider } from "@/plugins/socket";
import type { Socket } from "socket.io-client";
import { ROBOT_STATUS } from "@/common/constants";

let battery_charge_100 = 100;

export default defineComponent({
  name: "Home",
  data() {
    return {

      rover: ref(ROBOT_STATUS.offline),
      drone: ref(ROBOT_STATUS.offline),
      socket: inject(socketProvider) as Socket,
      mapImageUrl: ref(""),
      rover_battery: ref(battery_charge_100),
      drone_battery: ref(battery_charge_100),
    };
  },
  methods: {
    start() {
      this.socket.emit("start", { data: "Démarrer mission" });
      console.log("start");
    },
    identify() {
      this.socket.emit("identify", { data: "beep" });
    },
    finish() {
      this.socket.emit("finish", { data: "Finir mission" });
    },
    onRoverState(in_mission?: string) {
      if (in_mission === "True") {
        this.rover = ROBOT_STATUS.in_mission;
      } else if (in_mission === "False") {
        this.rover = ROBOT_STATUS.pending;
      } else if (in_mission === undefined) {
        this.rover = ROBOT_STATUS.offline;
      }
    },
    onDroneState(drone_in_mission?: string) {
      if (drone_in_mission === "True") {
        this.drone = ROBOT_STATUS.in_mission;
      } else if (drone_in_mission === "False") {
        this.drone = ROBOT_STATUS.pending;
      } else if (drone_in_mission === undefined) {
        this.drone = ROBOT_STATUS.offline;
      } else if (drone_in_mission === "Crashed") {
        this.drone = ROBOT_STATUS.crashed;
      }
    },
    onMapUpdate(map_bytes: ArrayBuffer) {
      const arrayBufferView = new Uint8Array(map_bytes);
      const blob = new Blob([arrayBufferView], { type: "image/png" });
      const imageUrl = URL.createObjectURL(blob);
      this.mapImageUrl = imageUrl;
      console.log(imageUrl);
    },
    return_home() {
      this.socket.emit("return_home", { data: "Retour à la base" });
    },
    onRoverBatteryState(battery_state: number) {
      this.rover_battery = battery_state;
    },
    onDroneBatteryState(battery_state: number) {
      this.drone_battery = battery_state;
    },
  },
  mounted() {
    this.socket.on("rover_state", this.onRoverState);
    this.socket.on("drone_state", this.onDroneState);
    this.socket.on("map_update", this.onMapUpdate);
    this.socket.on("rover_battery", this.onRoverBatteryState);
    this.socket.on("drone_battery", this.onDroneBatteryState);
  },
  unmounted() {
    this.socket.off("rover_state", this.onRoverState);
    this.socket.off("drone_state", this.onDroneState);
    this.socket.off("map_update", this.onMapUpdate);
    this.socket.off("rover_battery", this.onRoverBatteryState);
    this.socket.off("drone_battery", this.onDroneBatteryState);
  },
});
</script>

<template>
  <div class="home">
    <h1 id="title">Gestion de la mission</h1>
    <div id="buttons">
      <button
        v-show="rover !== 'en mission' && drone !== 'en mission'"
        @click="start"
      >
        Lancer
      </button>
      <button @click="identify">Identifier</button>
      <button
        v-show="rover == 'en mission' || drone == 'en mission'"
        @click="finish"
      >
        Terminer
      </button>
      <button @click="return_home">Retour à la Base</button>
    </div>
    <div>
      <span class="robot_state">Le rover est {{ rover }} </span>
      <span class="robot_state">Le drone est {{ drone }} </span>
    </div>
    <div>
      <span class="battery_state" v-if="drone !== 'hors ligne'"
        >La batterie du drone est à {{ drone_battery }}%
      </span>
      <span class="battery_state" v-if="rover !== 'hors ligne'"
        >La batterie du rover est à {{ rover_battery }}%
      </span>
    </div>
    <div class="image-container">
      <img
        v-if="mapImageUrl !== ''"
        class="image"
        :src="mapImageUrl"
        alt="Map"
      />
    </div>
  </div>
</template>

<style scoped>
.home {
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
}
.image-container {
  display: flex;
  justify-content: center;
}
.image {
  height: 50vh;
}
#title {
  color: #943e36;
  font-weight: bold;
  font-size: 50px;
  text-align: center;
}
#buttons {
  display: flex;
  gap: 25px;
}

.robot_state {
  margin-top: 10px;
  font-size: 20px;
  background-color: #d9c1b9;
  color: #604d44;
  padding: 10px;
  border-radius: 5px;
  box-shadow: 0px 0px 10px rgba(0, 0, 0, 0.2);
  width: 230px;
  float: left;
  position: relative;
  overflow: hidden;
  margin: 5px;
}

.battery_state {
  margin-top: 10px;
  font-size: 20px;
  background-color: #d9c1b9;
  color: #604d44;
  padding: 10px;
  border-radius: 5px;
  box-shadow: 0px 0px 10px rgba(0, 0, 0, 0.2);
  width: 230px;
  float: right;
  position: relative;
  overflow: hidden;
  margin: 5px;
}

span {
  display: block;
}

@media only screen and (max-width: 520px) {
  .image {
    height: 30vh;
  }
  #title {
    font-size: 30px;
    margin-top: 10px;
  }
  .btn {
    font-size: 16px;
    padding: 8px;
    margin: 0 3px;
  }
  .robot_state {
    font-size: 16px;
    width: 185px;
  }
  .battery_state {
    font-size: 16px;
    width: 185px;
  }
}
</style>
