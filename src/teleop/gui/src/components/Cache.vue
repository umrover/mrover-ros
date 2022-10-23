<template>
    <div>
        <h3>Cache Controls</h3>
        <button>Close cache</button>
        <button>Open cache</button>
        <div id="box-1">
            <div id="circle"></div>
            Site {{site}} Cache Open
        </div>
    </div>
</template>

<script>
export default {
    data() {
        return {
            site: "A"
        }
    }
}
</script>

<style scoped>
    #circle {
        height: 20px;
        width: 20px;
        border-radius: 50%;
        background-color: green;
        margin-right: 2%;
    }

    #box-1 {
        margin-top: 5%;
        display: flex;
        flex-direction: row;
    }

</style>
