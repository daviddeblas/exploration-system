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
      mapImageCogniflyUrl: ref(""),
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
    p2p() {
      this.socket.emit("p2p", { data: "activer le mode P2P" });
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
    },
    onMapCogniflyUpdate(map_bytes: ArrayBuffer) {
      const arrayBufferView = new Uint8Array(map_bytes);
      const blob = new Blob([arrayBufferView], { type: "image/png" });
      const imageUrl = URL.createObjectURL(blob);
      this.mapImageCogniflyUrl = imageUrl;
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
    this.socket.on("map_cognifly_update", this.onMapCogniflyUpdate);
  },
  unmounted() {
    this.socket.off("rover_state", this.onRoverState);
    this.socket.off("drone_state", this.onDroneState);
    this.socket.off("map_update", this.onMapUpdate);
    this.socket.off("rover_battery", this.onRoverBatteryState);
    this.socket.off("drone_battery", this.onDroneBatteryState);
    this.socket.off("map_cognifly_update", this.onMapCogniflyUpdate);
  },
});
</script>

<template>
  <div class="home">
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
      <button
        v-show="rover == 'en mission' || drone == 'en mission'"
        @click="p2p"
      >
        P2P
      </button>
      <button @click="return_home">Retour à la Base</button>
    </div>
    <div class="robotNav">
      <ul id="roverStatus">
        <li class="rover_status">Limo Status: {{ rover }}</li>
        <li v-if="rover !== 'hors ligne'" class="rover_battery">
          Limo Battery: {{ rover_battery }}%
        </li>
      </ul>
      <ul id="droneStatus">
        <li class="drone_status">Cognifly Status: {{ drone }}</li>
        <li v-if="drone !== 'hors ligne'" class="drone_battery">
          Cognifly Battery: {{ drone_battery }}%
        </li>
      </ul>
    </div>
    <div class="image-container">
      <img
        v-if="mapImageUrl !== ''"
        class="image"
        :src="mapImageUrl"
        alt="Map"
      />
    </div>
    <div class="image-container">
      <img
        v-if="mapImageCogniflyUrl !== ''"
        class="image"
        :src="mapImageCogniflyUrl"
        alt="CogniflyMap"
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

.robotNav {
  display: flex;
  font-size: 15px;
  justify-content: center;
  align-items: center;
}

#buttons {
  display: flex;
  align-items: center;
  justify-content: center;
}

ul li {
  text-decoration: underline;
  text-transform: none;
  background: #982a1c;
  border-color: #982a1c !important;
  color: white !important;
  transition: all 0.4s ease 0s;
  pointer-events: none;
}
span {
  display: block;
}
@media only screen and (max-width: 725px) {
  .image {
    height: 30vh;
    width: 100%;
    object-fit: contain;
  }
  #buttons {
    font-size: 16px;
    margin: 0px;
    display: block;
    text-align: center;
    width: auto;
  }
  .robotNav {
    display: block;
    justify-items: center;
  }
}
</style>
