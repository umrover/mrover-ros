<template>
<div>
    <div>
        <h3> Select Site </h3>
    </div>
    <div class="box1">
        <div>
            <label for="position">Change site to position:</label>
            <select v-model="site" name="site" id="site" @change="onChange($event)">
                <option value="A"> Position A </option>
                <option value="B"> Position B </option>
                <option value="C"> Position C </option>
            </select>
        </div>
    </div>
</div>
</template>

<script>
export default {
    data() {
        return {
            site: "A"
        }
    },

    components: {},

    methods: {
        onChange(event) {
            this.$emit('site', event.target.value)
        }
    }
}
</script>

<style scoped>
.box1 {
    text-align: left;
    vertical-align: top;
    display: inline-block;
}
</style>
