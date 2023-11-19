<template>
    <div>
        <div>
            <h2>Select Site</h2>
        </div>
        <div class="form-check form-check-inline">
            <input v-model="site" class="form-check-input" type="radio" name="flexRadioDefault" id="siteA" value="A">
            <label class="form-check-label" for="siteA">
                A
            </label>
        </div>
        <div class="form-check form-check-inline">
            <input v-model="site" class="form-check-input" type="radio" name="flexRadioDefault" id="siteB" value="B">
            <label class="form-check-label" for="siteB">
                B
            </label>
        </div>
        <div class="form-check form-check-inline">
            <input v-model="site" class="form-check-input" type="radio" name="flexRadioDefault" id="siteC" value="C">
            <label class="form-check-label" for="siteC">
                C
            </label>
        </div>
    </div>
</template>



<script lang="ts">
export default {
    components: {},
    data() {
        return {
            site: "A"
        };
    },

    methods: {
        onChange(event) {
            this.$emit("site", event.target.value);
        }
    }
};
</script>
  
<!-- <style scoped>
.radio-buttons {
    text-align: left;
    vertical-align: top;
    display: inline-block;
    font-size: larger;
}

.selected {
    padding: 3px;
    background-color: #4caf50;
    color: white;
    border: 1px solid black;
    border-radius: 5px;
}

.radio-buttons input {
    -ms-transform: scale(1.5);
    /* IE 9 */
    -webkit-transform: scale(1.5);
    /* Chrome, Safari, Opera */
    transform: scale(1.5);
}
</style> -->