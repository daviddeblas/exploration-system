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
  <div class="bottomNav">
    <ul id = "robotStatus">
      <li>Limo Status: {{ rover }}</li>
      <li>Cognifly Status: {{ drone }}</li>
    </ul>
    <ul id = battery>
      <li >Limo Battery: {{ rover_battery }}%</li>
      <li >Cognifly Battery: {{ drone_battery }}%</li>
    </ul>
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

.bottomNav {
  position: fixed;
  width: 100%;
  height: 120px;
  padding: 5px;
  bottom: 0%;
  display: flex;
  font-size: 25px;
}

#robotStatus{
  flex:1;
}

#battery{
  text-align: right;
  flex:1;
  margin-right: 35px;
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

ul li{
  text-decoration: underline;
  text-transform:none;
  background: #982a1c;
  border-color: #982a1c !important;
  color: white !important;
  transition: all 0.4s ease 0s;
  pointer-events: none;
}

/* ul il:hover{
  text-decoration: underline;
  background: #e65946;
  border-color: #e65946 !important;
  transition: all 0.4s ease 0s;
} */

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
