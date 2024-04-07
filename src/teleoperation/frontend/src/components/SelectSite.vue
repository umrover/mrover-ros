<template>
    <div>
        <div>
            <h2>Select Site</h2>
        </div>
        <div class="form-check form-check-inline">
            <input v-model="site" class="form-check-input" type="radio" name="flexRadioDefault" id="site0" value="0">
            <label class="form-check-label" for="site0">
                0
            </label>
        </div>
        <div class="form-check form-check-inline">
            <input v-model="site" class="form-check-input" type="radio" name="flexRadioDefault" id="site1" value="1">
            <label class="form-check-label" for="site1">
                1
            </label>
        </div>
        <div class="form-check form-check-inline">
            <input v-model="site" class="form-check-input" type="radio" name="flexRadioDefault" id="site2" value="2">
            <label class="form-check-label" for="site2">
                2
            </label>
        </div>
    </div>
</template>

<script lang="ts">
export default {
    components: {},
    data() {
        return {
            site: 0
        };
    },

    watch: {
        site(event) {
            this.$emit("site", event);
        }
    }
};
</script>
  
<style scoped>
</style>